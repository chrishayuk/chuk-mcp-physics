"""Physics analysis utilities for trajectories and events."""

import math
from typing import Optional

from .models import BounceEvent, ContactEvent, TrajectoryFrame, TrajectoryWithEventsResponse


def detect_bounces(
    frames: list[TrajectoryFrame],
    height_threshold: float = 0.01,
    velocity_threshold: float = 0.1,
    gravity_axis: int = 1,  # Y-axis by default
) -> list[BounceEvent]:
    """Detect bounce events from trajectory frames.

    A bounce is detected when:
    1. Vertical velocity changes from negative to positive (hitting ground)
    2. Height is below threshold (near ground)
    3. Velocity magnitude exceeds threshold (not at rest)

    Args:
        frames: Trajectory frames to analyze
        height_threshold: Maximum height to consider as "on ground" (meters)
        velocity_threshold: Minimum velocity to count as bounce (m/s)
        gravity_axis: Axis index for gravity direction (0=X, 1=Y, 2=Z)

    Returns:
        List of detected bounce events
    """
    if len(frames) < 3:
        return []

    bounces: list[BounceEvent] = []
    bounce_number = 0

    for i in range(1, len(frames) - 1):
        prev_frame = frames[i - 1]
        curr_frame = frames[i]
        next_frame = frames[i + 1]

        # Get velocities and positions
        if not prev_frame.velocity or not curr_frame.velocity or not next_frame.velocity:
            continue

        prev_vel_y = prev_frame.velocity[gravity_axis]
        next_vel_y = next_frame.velocity[gravity_axis]

        curr_height = curr_frame.position[gravity_axis]

        # Check for bounce: velocity goes from negative to positive
        if prev_vel_y < -velocity_threshold and next_vel_y > velocity_threshold:
            # Verify we're near the ground
            if curr_height < height_threshold:
                # Calculate speeds
                speed_before = _vector_magnitude(prev_frame.velocity)
                speed_after = _vector_magnitude(next_frame.velocity)

                # Calculate energy loss
                energy_before = 0.5 * speed_before**2  # Assuming unit mass
                energy_after = 0.5 * speed_after**2
                energy_loss_percent = (
                    ((energy_before - energy_after) / energy_before * 100)
                    if energy_before > 0
                    else 0.0
                )

                bounce_number += 1
                bounces.append(
                    BounceEvent(
                        time=curr_frame.time,
                        bounce_number=bounce_number,
                        position=curr_frame.position,
                        velocity_before=prev_frame.velocity,
                        velocity_after=next_frame.velocity,
                        height_at_bounce=curr_height,
                        speed_before=speed_before,
                        speed_after=speed_after,
                        energy_loss_percent=energy_loss_percent,
                    )
                )

    return bounces


def analyze_trajectory_with_events(
    frames: list[TrajectoryFrame],
    dt: float,
    body_id: str,
    total_time: float,
    detect_bounces_enabled: bool = True,
    bounce_height_threshold: float = 0.01,
    contact_events: Optional[list[ContactEvent]] = None,
) -> TrajectoryWithEventsResponse:
    """Analyze trajectory and detect events.

    Args:
        frames: Trajectory frames
        dt: Time step between frames
        body_id: Body identifier
        total_time: Total simulation time
        detect_bounces_enabled: Whether to detect bounce events
        bounce_height_threshold: Height threshold for bounce detection
        contact_events: Pre-detected contact events from simulation

    Returns:
        Trajectory with detected events
    """
    from .models import TrajectoryMeta

    # Detect bounces if enabled
    bounces: list[BounceEvent] = []
    if detect_bounces_enabled:
        bounces = detect_bounces(frames, height_threshold=bounce_height_threshold)

    # Use provided contact events or empty list
    contacts = contact_events or []

    return TrajectoryWithEventsResponse(
        dt=dt,
        frames=frames,
        meta=TrajectoryMeta(
            body_id=body_id,
            total_time=total_time,
            num_frames=len(frames),
        ),
        contact_events=contacts,
        bounces=bounces,
    )


def _vector_magnitude(vec: list[float]) -> float:
    """Calculate magnitude of a 3D vector."""
    return math.sqrt(sum(x * x for x in vec))


def count_bounces(
    trajectory: TrajectoryWithEventsResponse,
    min_height: Optional[float] = None,
) -> int:
    """Count number of bounces in trajectory.

    Args:
        trajectory: Trajectory with detected events
        min_height: Optional minimum height filter (only count bounces above this)

    Returns:
        Number of bounces
    """
    if min_height is None:
        return len(trajectory.bounces)

    return sum(1 for b in trajectory.bounces if b.height_at_bounce >= min_height)


def get_last_bounce_above_threshold(
    trajectory: TrajectoryWithEventsResponse,
    height_threshold: float,
) -> Optional[BounceEvent]:
    """Get the last bounce that occurred above a height threshold.

    Useful for answering "when did the ball stop bouncing above 5mm?"

    Args:
        trajectory: Trajectory with detected events
        height_threshold: Minimum height threshold

    Returns:
        Last bounce above threshold, or None if no bounces above threshold
    """
    above_threshold = [b for b in trajectory.bounces if b.height_at_bounce >= height_threshold]
    return above_threshold[-1] if above_threshold else None
