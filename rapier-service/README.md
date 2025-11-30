# Rapier Physics Service

High-performance Rust microservice providing rigid-body physics simulations via HTTP API.

## 🚀 Quick Start

### Using Docker (Recommended)

```bash
# Build image
docker build -t rapier-physics-service .

# Run service
docker run -p 9000:9000 rapier-physics-service
```

### From Source

Requirements:
- Rust 1.83+ (`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`)

```bash
# Build
cargo build --release

# Run
cargo run --release
```

The service will start on `http://localhost:9000`

## 🧪 Testing

```bash
# Health check
curl http://localhost:9000/health

# Create simulation
curl -X POST http://localhost:9000/simulations \
  -H "Content-Type: application/json" \
  -d '{
    "gravity": [0, -9.81, 0],
    "dimensions": 3,
    "dt": 0.016
  }'

# Response: {"sim_id":"123e4567-e89b-12d3-a456-426614174000","config":{...}}
```

## 📡 API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Health check |
| `/simulations` | POST | Create new simulation |
| `/simulations/:id` | GET | Get simulation state |
| `/simulations/:id` | DELETE | Destroy simulation |
| `/simulations/:id/bodies` | POST | Add rigid body |
| `/simulations/:id/step` | POST | Step simulation |
| `/simulations/:id/record/:body_id` | POST | Record trajectory |

See [RAPIER_SERVICE.md](../RAPIER_SERVICE.md) for full API documentation.

## 🏗️ Architecture

- **Axum** - Fast async web framework
- **Rapier3D** - Physics engine (Rust)
- **Tokio** - Async runtime
- **Tower** - Middleware (CORS, logging)

## 🔧 Configuration

Environment variables:

- `RUST_LOG` - Logging level (default: `info`)
  - Example: `RUST_LOG=debug cargo run`

## 📊 Performance

- **Request latency**: < 1ms for simple simulations
- **Throughput**: 10,000+ steps/second (single simulation)
- **Memory**: ~10MB base + ~1KB per rigid body
- **Concurrency**: Unlimited concurrent simulations (memory permitting)

## 🐳 Docker Deployment

### Fly.io

```bash
# Deploy
fly deploy

# Scale
fly scale count 2

# Logs
fly logs
```

### Docker Compose

```yaml
services:
  rapier:
    build: .
    ports:
      - "9000:9000"
    environment:
      - RUST_LOG=info
```

## 🧑‍💻 Development

```bash
# Watch mode (requires cargo-watch)
cargo install cargo-watch
cargo watch -x run

# Format
cargo fmt

# Lint
cargo clippy

# Test
cargo test
```

## 📝 License

MIT - Same as parent project
