"""Shared pytest fixtures for physics MCP server tests."""

import pytest
from typing import Callable
import asyncio


@pytest.fixture
def sample_position() -> list[float]:
    """Sample 3D position."""
    return [0.0, 0.0, 0.0]


@pytest.fixture
def sample_velocity() -> list[float]:
    """Sample 3D velocity."""
    return [10.0, 0.0, 0.0]


@pytest.fixture
def earth_gravity() -> float:
    """Standard Earth gravity."""
    return 9.81


@pytest.fixture
def projectile_45deg() -> dict:
    """Standard 45-degree projectile test case."""
    return {
        "initial_velocity": 20.0,
        "angle_degrees": 45.0,
        "initial_height": 0.0,
        "gravity": 9.81,
    }


@pytest.fixture
def collision_scenario() -> dict:
    """Two spheres on collision course."""
    return {
        "body1_position": [0.0, 0.0, 0.0],
        "body1_velocity": [10.0, 0.0, 0.0],
        "body1_radius": 1.0,
        "body2_position": [20.0, 0.0, 0.0],
        "body2_velocity": [-10.0, 0.0, 0.0],
        "body2_radius": 1.0,
        "max_time": 10.0,
    }


@pytest.fixture
def no_collision_scenario() -> dict:
    """Two spheres that miss each other."""
    return {
        "body1_position": [0.0, 0.0, 0.0],
        "body1_velocity": [10.0, 0.0, 0.0],
        "body1_radius": 1.0,
        "body2_position": [0.0, 10.0, 0.0],
        "body2_velocity": [10.0, 0.0, 0.0],
        "body2_radius": 1.0,
        "max_time": 10.0,
    }


@pytest.fixture
async def retry_on_error() -> Callable:
    """Retry wrapper for flaky tests (e.g., network)."""

    async def retry(func: Callable, max_retries: int = 3, delay: float = 1.0):
        """Retry async function up to max_retries times."""
        last_error = None
        for attempt in range(max_retries):
            try:
                return await func()
            except Exception as e:
                last_error = e
                if attempt < max_retries - 1:
                    await asyncio.sleep(delay)
        raise last_error

    return retry
