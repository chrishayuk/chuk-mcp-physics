"""Provider factory for creating physics providers.

This module provides a factory pattern for instantiating providers
based on configuration.
"""

import logging
from enum import Enum

from ..config import ProviderConfig
from .base import PhysicsProvider

logger = logging.getLogger(__name__)


class ProviderType(str, Enum):
    """Available provider types."""

    ANALYTIC = "analytic"
    RAPIER = "rapier"


# Cache provider instances for reuse
_provider_cache: dict[str, PhysicsProvider] = {}


def get_provider(provider_type: str | None = None) -> PhysicsProvider:
    """Get a provider instance based on type.

    Args:
        provider_type: Type of provider to create. If None, uses default from config.

    Returns:
        PhysicsProvider instance

    Raises:
        ValueError: If provider_type is invalid or provider cannot be created
    """
    if provider_type is None:
        provider_type = ProviderConfig.DEFAULT_PROVIDER

    # Return cached instance if available
    if provider_type in _provider_cache:
        return _provider_cache[provider_type]

    # Create new provider instance
    provider: PhysicsProvider
    try:
        if provider_type == ProviderType.ANALYTIC.value:
            from .analytic import AnalyticProvider

            provider = AnalyticProvider()
            logger.info("Created Analytic provider")

        elif provider_type == ProviderType.RAPIER.value:
            from .rapier import RapierProvider

            provider = RapierProvider()
            logger.info("Created Rapier provider")

        else:
            raise ValueError(
                f"Unknown provider type: {provider_type}. "
                f"Available: {[p.value for p in ProviderType]}"
            )

        # Cache the provider
        _provider_cache[provider_type] = provider
        return provider

    except ImportError as e:
        raise ValueError(f"Provider '{provider_type}' requires additional dependencies: {e}") from e


def get_provider_for_tool(tool_name: str) -> PhysicsProvider:
    """Get the configured provider for a specific tool.

    This allows per-tool provider configuration.

    Args:
        tool_name: Name of the tool (projectile_motion, collision_check, etc.)

    Returns:
        PhysicsProvider instance configured for this tool
    """
    provider_type = None

    # Map tool names to config attributes
    tool_config_map = {
        "projectile_motion": ProviderConfig.PROJECTILE_MOTION_PROVIDER,
        "collision_check": ProviderConfig.COLLISION_CHECK_PROVIDER,
        "force_calculation": ProviderConfig.FORCE_CALCULATION_PROVIDER,
        "kinetic_energy": ProviderConfig.KINETIC_ENERGY_PROVIDER,
        "momentum": ProviderConfig.MOMENTUM_PROVIDER,
        # Simulation tools
        "create_simulation": ProviderConfig.SIMULATION_PROVIDER,
        "add_body": ProviderConfig.SIMULATION_PROVIDER,
        "step_simulation": ProviderConfig.SIMULATION_PROVIDER,
        "get_simulation_state": ProviderConfig.SIMULATION_PROVIDER,
        "record_trajectory": ProviderConfig.SIMULATION_PROVIDER,
        "destroy_simulation": ProviderConfig.SIMULATION_PROVIDER,
    }

    provider_type = tool_config_map.get(tool_name)

    if provider_type is None:
        logger.warning(f"No specific provider configured for tool '{tool_name}', using default")
        provider_type = ProviderConfig.DEFAULT_PROVIDER

    return get_provider(provider_type)


def clear_provider_cache() -> None:
    """Clear the provider cache.

    Useful for testing or forcing provider re-initialization.
    """
    global _provider_cache
    _provider_cache.clear()
    logger.debug("Provider cache cleared")
