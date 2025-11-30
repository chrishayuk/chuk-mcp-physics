"""Configuration for chuk-mcp-physics server.

This module handles configuration loading from YAML files and environment variables.
Configuration sources (in order of precedence):
1. Environment variables
2. YAML configuration file (physics.yaml)
3. Default values
"""

import logging
import os
from enum import Enum
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Try to import yaml, but don't fail if not available
try:
    import yaml  # type: ignore[import-untyped]

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False
    logger.debug("PyYAML not available, using environment variables only")


def load_yaml_config() -> dict[str, Any]:
    """Load configuration from YAML file if it exists.

    Looks for physics.yaml in:
    1. Current working directory
    2. User's home directory (~/.config/chuk-mcp-physics/)
    3. Environment variable PHYSICS_CONFIG_PATH

    Returns:
        Dictionary of configuration values, or empty dict if no file found
    """
    if not YAML_AVAILABLE:
        return {}

    config_paths = [
        Path.cwd() / "physics.yaml",
        Path.home() / ".config" / "chuk-mcp-physics" / "physics.yaml",
    ]

    # Add path from environment variable if set
    if env_path := os.getenv("PHYSICS_CONFIG_PATH"):
        config_paths.insert(0, Path(env_path))

    for config_path in config_paths:
        if config_path.exists():
            try:
                with open(config_path) as f:
                    config = yaml.safe_load(f)
                    logger.info(f"Loaded configuration from {config_path}")
                    return config or {}
            except Exception as e:
                logger.warning(f"Failed to load config from {config_path}: {e}")

    logger.debug("No YAML config file found, using environment variables and defaults")
    return {}


class ProviderType(str, Enum):
    """Available provider types."""

    ANALYTIC = "analytic"
    RAPIER = "rapier"


# Load YAML config once at module import
_yaml_config = load_yaml_config()


class ProviderConfig:
    """Configuration for provider selection.

    Providers can be configured globally or per-tool via:
    1. Environment variables (highest priority)
    2. YAML configuration file
    3. Default values

    Example YAML:
        default_provider: analytic
        providers:
          projectile_motion: analytic
          simulations: rapier
    """

    # Default provider for all tools
    # Priority: env var > YAML > hardcoded default
    DEFAULT_PROVIDER = os.getenv(
        "PHYSICS_PROVIDER",
        _yaml_config.get("default_provider", ProviderType.ANALYTIC.value),
    )

    # Per-tool provider overrides
    _providers_yaml = _yaml_config.get("providers", {})

    # Analytic calculations can only use analytic provider
    PROJECTILE_MOTION_PROVIDER = ProviderType.ANALYTIC.value
    COLLISION_CHECK_PROVIDER = ProviderType.ANALYTIC.value
    FORCE_CALCULATION_PROVIDER = ProviderType.ANALYTIC.value
    KINETIC_ENERGY_PROVIDER = ProviderType.ANALYTIC.value
    MOMENTUM_PROVIDER = ProviderType.ANALYTIC.value

    # Simulation tools use configured provider
    SIMULATION_PROVIDER = os.getenv(
        "PHYSICS_SIMULATION_PROVIDER",
        _providers_yaml.get("simulations", DEFAULT_PROVIDER),
    )


class RapierConfig:
    """Configuration for Rapier service provider.

    Loads from (in order of precedence):
    1. Environment variables
    2. YAML config rapier section
    3. Default values
    """

    _rapier_yaml = _yaml_config.get("rapier", {})

    # Rapier service URL
    SERVICE_URL = os.getenv(
        "RAPIER_SERVICE_URL",
        _rapier_yaml.get("service_url", "http://localhost:9000"),
    )

    # Request timeout in seconds
    TIMEOUT = float(
        os.getenv(
            "RAPIER_TIMEOUT",
            str(_rapier_yaml.get("timeout", 30.0)),
        )
    )

    # Retry configuration
    MAX_RETRIES = int(
        os.getenv(
            "RAPIER_MAX_RETRIES",
            str(_rapier_yaml.get("max_retries", 3)),
        )
    )
    RETRY_DELAY = float(
        os.getenv(
            "RAPIER_RETRY_DELAY",
            str(_rapier_yaml.get("retry_delay", 1.0)),
        )
    )


class SimulationLimits:
    """Safety limits for physics simulations.

    These limits prevent timeouts, instabilities, and resource exhaustion.
    They can be overridden via environment variables if needed.
    """

    # General limits
    MAX_STEPS_PER_CALL = int(os.getenv("PHYSICS_MAX_STEPS", "10000"))
    MAX_BODIES_PER_SIM = int(os.getenv("PHYSICS_MAX_BODIES", "1000"))
    MAX_DURATION = float(os.getenv("PHYSICS_MAX_DURATION", "60.0"))
    MAX_TRAJECTORY_FRAMES = int(os.getenv("PHYSICS_MAX_TRAJECTORY_FRAMES", "10000"))

    # Timestep constraints (seconds)
    MIN_DT = float(os.getenv("PHYSICS_MIN_DT", "0.001"))
    MAX_DT = float(os.getenv("PHYSICS_MAX_DT", "0.1"))
    RECOMMENDED_DT = 0.016  # 60 FPS

    # Public service limits (stricter for shared infrastructure)
    PUBLIC_MAX_STEPS = 5_000
    PUBLIC_MAX_BODIES = 100
    PUBLIC_MAX_CONCURRENT_SIMS = 10
    PUBLIC_TIMEOUT = 30.0
