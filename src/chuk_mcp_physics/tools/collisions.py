"""Collision calculation MCP tool endpoints."""

import json
from typing import Union

from chuk_mcp_server import tool


@tool  # type: ignore[arg-type]
async def calculate_inelastic_collision_3d(
    mass1: float,
    velocity1: Union[list[float], str],
    mass2: float,
    velocity2: Union[list[float], str],
    coefficient_of_restitution: float = 0.0,
) -> dict:
    """Calculate 3D collision with coefficient of restitution.

    Models realistic collisions where some kinetic energy is lost.
    Coefficient of restitution (e) determines how much energy is retained.

    Args:
        mass1: Mass of object 1 in kg
        velocity1: Velocity of object 1 [x, y, z] in m/s (or JSON string)
        mass2: Mass of object 2 in kg
        velocity2: Velocity of object 2 [x, y, z] in m/s (or JSON string)
        coefficient_of_restitution: e (0.0 = perfectly inelastic, 1.0 = perfectly elastic)

    Returns:
        Dict containing:
            - final_velocity1: Final velocity [x, y, z] in m/s
            - final_velocity2: Final velocity [x, y, z] in m/s
            - initial_momentum: Total initial momentum [x, y, z]
            - final_momentum: Total final momentum [x, y, z]
            - initial_kinetic_energy: Total initial KE in Joules
            - final_kinetic_energy: Total final KE in Joules
            - energy_loss: Energy lost in Joules
            - energy_loss_percent: % of energy lost

    Coefficient of restitution values:
        - e = 0.0: Perfectly inelastic (clay, putty) - objects stick
        - e = 0.5: Very inelastic (wet clay)
        - e = 0.7: Moderately elastic (basketball)
        - e = 0.9: Highly elastic (Super Ball)
        - e = 1.0: Perfectly elastic (ideal, no energy loss)

    Tips for LLMs:
        - Momentum is always conserved (regardless of e)
        - Energy lost = (1 - e²) × initial KE in center-of-mass frame
        - Use e=1.0 for billiard balls, e=0.0 for car crashes

    Example - Car crash:
        result = await calculate_inelastic_collision_3d(
            mass1=1500,  # kg
            velocity1=[20, 0, 0],  # 20 m/s east
            mass2=1200,  # kg
            velocity2=[-15, 0, 0],  # 15 m/s west
            coefficient_of_restitution=0.1  # very inelastic
        )
        # Massive energy loss, objects nearly stick together
    """
    # Parse inputs
    parsed_v1 = json.loads(velocity1) if isinstance(velocity1, str) else velocity1
    parsed_v2 = json.loads(velocity2) if isinstance(velocity2, str) else velocity2

    from ..collisions import (
        InelasticCollision3DRequest,
        calculate_inelastic_collision_3d as calc_coll,
    )

    request = InelasticCollision3DRequest(
        mass1=mass1,
        velocity1=parsed_v1,
        mass2=mass2,
        velocity2=parsed_v2,
        coefficient_of_restitution=coefficient_of_restitution,
    )
    response = calc_coll(request)
    return response.model_dump()


@tool  # type: ignore[arg-type]
async def calculate_elastic_collision_3d(
    mass1: float,
    velocity1: Union[list[float], str],
    mass2: float,
    velocity2: Union[list[float], str],
) -> dict:
    """Calculate 3D elastic collision (perfect energy conservation).

    Special case of collision where no kinetic energy is lost (e = 1.0).
    Both momentum and energy are conserved.

    Args:
        mass1: Mass of object 1 in kg
        velocity1: Velocity of object 1 [x, y, z] in m/s (or JSON string)
        mass2: Mass of object 2 in kg
        velocity2: Velocity of object 2 [x, y, z] in m/s (or JSON string)

    Returns:
        Dict containing:
            - final_velocity1: Final velocity [x, y, z] in m/s
            - final_velocity2: Final velocity [x, y, z] in m/s
            - initial_momentum: Total momentum [x, y, z]
            - final_momentum: Total momentum [x, y, z]
            - initial_kinetic_energy: Total KE in Joules
            - final_kinetic_energy: Total KE in Joules

    Tips for LLMs:
        - Ideal approximation for billiard balls, Newton's cradle
        - Both momentum and energy conserved
        - Equal masses + head-on → velocities exchange
        - Use for educational examples, idealized systems

    Example - Pool balls:
        result = await calculate_elastic_collision_3d(
            mass1=0.17,  # kg (pool ball)
            velocity1=[2, 0, 0],  # 2 m/s
            mass2=0.17,  # kg
            velocity2=[0, 0, 0]  # stationary
        )
        # Result: ball 1 stops, ball 2 moves at 2 m/s
    """
    # Parse inputs
    parsed_v1 = json.loads(velocity1) if isinstance(velocity1, str) else velocity1
    parsed_v2 = json.loads(velocity2) if isinstance(velocity2, str) else velocity2

    from ..collisions import (
        ElasticCollision3DRequest,
        calculate_elastic_collision_3d as calc_elastic,
    )

    request = ElasticCollision3DRequest(
        mass1=mass1,
        velocity1=parsed_v1,
        mass2=mass2,
        velocity2=parsed_v2,
    )
    response = calc_elastic(request)
    return response.model_dump()
