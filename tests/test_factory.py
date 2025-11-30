"""Tests for provider factory."""

import pytest
from chuk_mcp_physics.providers.factory import (
    get_provider,
    get_provider_for_tool,
    clear_provider_cache,
    ProviderType,
)
from chuk_mcp_physics.providers.base import PhysicsProvider
from chuk_mcp_physics.providers.analytic import AnalyticProvider


@pytest.fixture(autouse=True)
def clear_cache():
    """Clear provider cache before each test."""
    clear_provider_cache()
    yield
    clear_provider_cache()


class TestGetProvider:
    """Test get_provider function."""

    def test_get_analytic_provider(self):
        """Get analytic provider by name."""
        provider = get_provider(ProviderType.ANALYTIC.value)
        assert isinstance(provider, AnalyticProvider)
        assert provider.name == "analytic"

    def test_get_default_provider(self):
        """Get default provider (None argument)."""
        provider = get_provider(None)
        assert isinstance(provider, PhysicsProvider)

    def test_provider_caching(self):
        """Providers should be cached."""
        provider1 = get_provider(ProviderType.ANALYTIC.value)
        provider2 = get_provider(ProviderType.ANALYTIC.value)
        assert provider1 is provider2  # Same instance

    def test_invalid_provider_type(self):
        """Invalid provider type should raise ValueError."""
        with pytest.raises(ValueError, match="Unknown provider type"):
            get_provider("invalid_provider")

    def test_clear_cache(self):
        """Cache clearing should work."""
        provider1 = get_provider(ProviderType.ANALYTIC.value)
        clear_provider_cache()
        provider2 = get_provider(ProviderType.ANALYTIC.value)
        assert provider1 is not provider2  # Different instances after clear


class TestGetProviderForTool:
    """Test get_provider_for_tool function."""

    def test_projectile_motion_tool(self):
        """Projectile motion always uses analytic."""
        provider = get_provider_for_tool("projectile_motion")
        assert isinstance(provider, AnalyticProvider)

    def test_collision_check_tool(self):
        """Collision check always uses analytic."""
        provider = get_provider_for_tool("collision_check")
        assert isinstance(provider, AnalyticProvider)

    def test_force_calculation_tool(self):
        """Force calculation always uses analytic."""
        provider = get_provider_for_tool("force_calculation")
        assert isinstance(provider, AnalyticProvider)

    def test_simulation_tool(self):
        """Simulation tools use configured provider."""
        provider = get_provider_for_tool("create_simulation")
        assert isinstance(provider, PhysicsProvider)

    def test_unknown_tool(self):
        """Unknown tool should use default provider."""
        provider = get_provider_for_tool("unknown_tool")
        assert isinstance(provider, PhysicsProvider)
