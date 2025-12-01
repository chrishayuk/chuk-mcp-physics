# Fly.io Redis Setup for Rapier Physics Service

This guide explains how to set up Redis for the Rapier physics service on Fly.io.

## Why Redis?

The Rapier service uses Redis for:
- **Horizontal scaling**: Multiple instances can run simultaneously
- **Automatic cleanup**: TTL ensures abandoned simulations don't leak memory
- **Distributed coordination**: Simulations tracked across all instances
- **Session persistence**: Simulations survive service restarts (within TTL)

## Quick Setup (5 minutes)

### 1. Create a Fly Redis Instance

```bash
# Navigate to rapier-service directory
cd rapier-service

# Create a Redis instance in the same region as your app
fly redis create

# When prompted:
# - Name: chuk-rapier-redis (or your preferred name)
# - Region: sjc (or same as your app's primary_region)
# - Plan: 256MB is sufficient for development
# - Eviction: allkeys-lru (recommended)
```

**Output example:**
```
? Select Organization: Your Org
? Choose a primary region (can't be changed later) San Jose, California (US) (sjc)
? Optionally, choose one or more replica regions (can be changed later):
? Select an Upstash Redis plan Free: 256 MB Max Data Size

Your Upstash Redis database chuk-rapier-redis is ready.

Apps in the personal org can connect to it at:
  redis://default:password@fly-chuk-rapier-redis.upstash.io

Set one or more of the following secrets on your target app:
  REDIS_URL: redis://default:password@fly-chuk-rapier-redis.upstash.io
```

### 2. Set the Redis URL Secret

```bash
# Set the REDIS_URL secret (use the URL from step 1)
fly secrets set REDIS_URL="redis://default:YOUR_PASSWORD@fly-chuk-rapier-redis.upstash.io"

# Verify the secret was set
fly secrets list
```

### 3. Deploy the Service

```bash
# Deploy with Redis enabled
fly deploy

# Check logs to verify Redis connection
fly logs
```

You should see in the logs:
```
📦 Initialized RedisStorage backend
   URL: redis://default:****@fly-chuk-rapier-redis.upstash.io
   TTL: 3600 seconds
```

## Configuration

The service automatically uses Redis when deployed to Fly.io. Configuration is done via environment variables in `fly.toml`:

```toml
[env]
  STORAGE_BACKEND = "redis"           # Use Redis storage
  STORAGE_TTL_SECONDS = "3600"        # 1 hour simulation lifetime
```

And secrets (set via `fly secrets set`):
```bash
REDIS_URL="redis://default:password@your-redis-url"
```

### TTL Configuration

Adjust `STORAGE_TTL_SECONDS` based on your use case:

| Use Case | Recommended TTL | Reasoning |
|----------|----------------|-----------|
| Quick calculations | 300 (5 min) | Short-lived, immediate results |
| Interactive demos | 1800 (30 min) | User exploration |
| Video generation | 3600 (1 hour) | Rendering takes time |
| Long simulations | 7200 (2 hours) | Complex multi-step workflows |

**Note:** Longer TTLs use more Redis memory. Monitor usage in Fly dashboard.

## Local Development

When running locally, the service automatically uses **in-memory storage** (no Redis needed):

```bash
# Local development (no Redis required)
cargo run

# Output:
# 📦 Initialized MemoryStorage backend (local development)
```

To test Redis locally:

```bash
# Start local Redis
docker run -d -p 6379:6379 redis:7-alpine

# Set environment variables
export STORAGE_BACKEND=redis
export REDIS_URL=redis://127.0.0.1:6379

# Run service
cargo run
```

## Monitoring

### Check Redis Usage

```bash
# View Redis metrics in Fly dashboard
fly dashboard

# Or use Upstash CLI
fly redis connect
> INFO memory
> KEYS sim:*:meta
```

### Simulation Statistics

The service automatically logs:
- Simulation creation/destruction
- Redis connection status
- TTL refresh operations

```bash
# Watch logs in real-time
fly logs

# Example output:
# Created simulation abc-123
# 📦 Redis metadata stored for abc-123
# Destroyed simulation abc-123
```

## Troubleshooting

### Redis Connection Fails

**Problem:** Service can't connect to Redis

**Solution:**
1. Verify REDIS_URL secret is set: `fly secrets list`
2. Check Redis instance is running: `fly redis status`
3. Ensure app and Redis are in same organization
4. Check logs for connection errors: `fly logs`

### Simulations Expiring Too Quickly

**Problem:** Simulations disappear before work is done

**Solution:**
1. Increase `STORAGE_TTL_SECONDS` in fly.toml
2. Redeploy: `fly deploy`
3. Note: Each access refreshes the TTL automatically

### High Redis Memory Usage

**Problem:** Redis approaching memory limit

**Solution:**
1. Reduce `STORAGE_TTL_SECONDS` for faster cleanup
2. Upgrade Redis plan: `fly redis update`
3. Check for simulation leaks (not calling destroy)

## Scaling

### Horizontal Scaling

Redis enables multiple Rapier instances:

```bash
# Scale to 3 instances
fly scale count 3

# All instances share the same Redis
# Simulations accessible from any instance
```

### Vertical Scaling

Increase Redis memory for more simulations:

```bash
# Upgrade to larger Redis plan
fly redis update --plan <plan-name>

# Available plans:
# - Free: 256 MB
# - Paid: 1 GB, 5 GB, etc.
```

## Production Best Practices

1. **Set appropriate TTL**: Balance between memory usage and user experience
2. **Monitor Redis metrics**: Watch memory, connections, evictions
3. **Use secrets for URLs**: Never commit Redis URLs to git
4. **Enable eviction**: Use `allkeys-lru` policy (default)
5. **Regular cleanup**: Implement periodic garbage collection if needed

## Cost Optimization

**Free Tier (256 MB Redis):**
- ~1000-5000 simulations (depending on complexity)
- Perfect for demos and development
- Automatic eviction when full

**Paid Tiers:**
- Scale based on simulation volume
- Monitor actual usage first
- Upgrade only when needed

## Architecture

```
┌─────────────────────────────────────────────┐
│ Fly.io App: chuk-rapier-physics             │
│                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │Instance 1│  │Instance 2│  │Instance 3│  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  │
│       │             │             │         │
│       └─────────────┼─────────────┘         │
│                     │                       │
└─────────────────────┼───────────────────────┘
                      │
                      ▼
          ┌───────────────────────┐
          │ Fly Redis             │
          │ chuk-rapier-redis     │
          │                       │
          │ sim:abc-123:meta      │
          │ sim:def-456:meta      │
          │ sim:ghi-789:meta      │
          └───────────────────────┘
```

Each instance:
- Stores simulation state in memory
- Tracks metadata + TTL in Redis
- Refreshes TTL on each access
- Cleans up expired sims automatically

## Summary

✅ **Automatic**: Redis setup is just 3 commands
✅ **Scalable**: Add more instances anytime
✅ **Cost-effective**: Free tier sufficient for most cases
✅ **Zero-config locally**: Memory storage for development

Redis integration makes the Rapier service production-ready! 🚀
