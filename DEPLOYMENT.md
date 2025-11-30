# Deployment Guide

This guide covers deploying the physics MCP server in production with separate Rapier service deployment.

## Architecture

```
┌─────────────────┐
│  MCP Client     │ (Claude Desktop, etc.)
│  (Claude Code)  │
└────────┬────────┘
         │
         │ MCP Protocol
         │
┌────────▼────────────┐
│  chuk-mcp-physics   │ (Python - Fly.io App 1)
│  MCP Server         │
└────────┬────────────┘
         │
         │ HTTP REST API
         │
┌────────▼─────────────┐
│  rapier-physics      │ (Rust - Fly.io App 2)
│  Service             │
└──────────────────────┘
```

## Option 1: Separate Deployment (Recommended)

### Benefits
- ✅ Independent scaling
- ✅ Language isolation (Python + Rust)
- ✅ Faster deployments
- ✅ Cost optimization
- ✅ Service reusability

### Step 1: Deploy Rapier Service

```bash
cd rapier-service

# Login to Fly.io
fly auth login

# Create app (first time only)
fly apps create chuk-rapier-physics

# Deploy
fly deploy

# Check status
fly status

# View logs
fly logs
```

**Result:** Rapier service running at `https://chuk-rapier-physics.fly.dev`

### Step 2: Deploy MCP Server

```bash
cd ..  # Back to project root

# Create app (first time only)
fly apps create chuk-mcp-physics

# Set Rapier service URL
fly secrets set RAPIER_SERVICE_URL=https://chuk-rapier-physics.fly.dev

# Deploy
fly deploy

# Check status
fly status
```

### Step 3: Configure Client

Update your `physics.yaml` or set environment variables:

```yaml
default_provider: rapier

rapier:
  service_url: https://chuk-rapier-physics.fly.dev
  timeout: 30.0
```

Or use environment variables:

```bash
export PHYSICS_PROVIDER=rapier
export RAPIER_SERVICE_URL=https://chuk-rapier-physics.fly.dev
```

## Option 2: All-in-One Deployment

If you prefer to run everything in one container (not recommended for production):

### Modify Dockerfile

Add Rust build stage and include Rapier binary:

```dockerfile
FROM rust:1.75-slim as rust-builder
WORKDIR /app
COPY rapier-service/ ./rapier-service/
RUN cd rapier-service && cargo build --release

FROM python:3.11-slim
# ... copy both Python app and Rust binary
# ... run both services with supervisor/process manager
```

**Downsides:**
- Slower builds (rebuild both languages every time)
- Larger images
- No independent scaling
- More complex health checks

## Scaling Configuration

### Rapier Service (CPU-Intensive)

```bash
# Scale up for physics workloads
fly scale count 2 -a chuk-rapier-physics
fly scale vm shared-cpu-2x -a chuk-rapier-physics

# Auto-scaling
fly autoscale set min=1 max=5 -a chuk-rapier-physics
```

### MCP Server (Lightweight)

```bash
# Minimal resources needed
fly scale vm shared-cpu-1x -a chuk-mcp-physics
fly autoscale set min=0 max=2 -a chuk-mcp-physics
```

## Cost Optimization

### Free Tier Strategy

Fly.io free tier: 3 shared-cpu-1x VMs

**Option A: Auto-sleep both services**
```toml
# In both fly.toml files
[http_service]
  auto_stop_machines = true
  auto_start_machines = true
  min_machines_running = 0
```

**Option B: Keep MCP server warm, sleep Rapier**
- MCP server: `min_machines_running = 1`
- Rapier service: `min_machines_running = 0`

## Health Checks

### Rapier Service

```bash
curl https://chuk-rapier-physics.fly.dev/health
# {"status":"healthy","service":"rapier-physics-service","version":"0.1.0"}
```

### MCP Server

```bash
# MCP servers don't have HTTP endpoints by default
# Use Fly.io health checks based on process
```

## Monitoring

```bash
# Rapier service metrics
fly metrics -a chuk-rapier-physics

# MCP server metrics
fly metrics -a chuk-mcp-physics

# Combined logging
fly logs -a chuk-rapier-physics &
fly logs -a chuk-mcp-physics
```

## CI/CD with GitHub Actions

### Separate Workflows

**.github/workflows/deploy-rapier.yml**
```yaml
name: Deploy Rapier Service

on:
  push:
    branches: [main]
    paths:
      - 'rapier-service/**'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: superfly/flyctl-actions/setup-flyctl@master
      - run: flyctl deploy --remote-only
        working-directory: ./rapier-service
        env:
          FLY_API_TOKEN: ${{ secrets.FLY_API_TOKEN }}
```

**.github/workflows/deploy-mcp.yml**
```yaml
name: Deploy MCP Server

on:
  push:
    branches: [main]
    paths:
      - 'src/**'
      - 'Dockerfile'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: superfly/flyctl-actions/setup-flyctl@master
      - run: flyctl deploy --remote-only
        env:
          FLY_API_TOKEN: ${{ secrets.FLY_API_TOKEN }}
```

## Local Development

### Docker Compose

```yaml
version: '3.8'

services:
  rapier:
    build: ./rapier-service
    ports:
      - "9000:9000"
    environment:
      - RUST_LOG=info

  mcp:
    build: .
    environment:
      - RAPIER_SERVICE_URL=http://rapier:9000
      - PHYSICS_PROVIDER=rapier
    depends_on:
      - rapier
```

Run with:
```bash
docker-compose up
```

## Troubleshooting

### Rapier service not reachable

```bash
# Check Rapier service is running
fly status -a chuk-rapier-physics

# Test endpoint directly
curl https://chuk-rapier-physics.fly.dev/health

# Check MCP server environment
fly ssh console -a chuk-mcp-physics
echo $RAPIER_SERVICE_URL
```

### High latency

- Ensure both apps in same region: `fly regions list`
- Use Fly.io private network (6PN) for internal communication
- Consider caching simulation results

### Out of memory

```bash
# Increase Rapier service memory
fly scale memory 512 -a chuk-rapier-physics

# Check actual usage
fly metrics -a chuk-rapier-physics
```

## Security

### Private Rapier Service

Make Rapier service only accessible internally:

```toml
# rapier-service/fly.toml
[http_service]
  internal_port = 9000
  # Remove public ports - only accessible via 6PN
```

Then use internal DNS:
```bash
fly secrets set RAPIER_SERVICE_URL=http://chuk-rapier-physics.internal:9000 -a chuk-mcp-physics
```

## Production Checklist

- [ ] Rapier service deployed and healthy
- [ ] MCP server deployed with correct RAPIER_SERVICE_URL
- [ ] Health checks passing
- [ ] Auto-scaling configured
- [ ] Monitoring set up
- [ ] CI/CD workflows tested
- [ ] Client configuration updated
- [ ] Load testing completed
- [ ] Cost monitoring enabled
- [ ] Backup strategy defined
