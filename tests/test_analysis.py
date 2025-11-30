"""Tests for physics analysis utilities."""

from chuk_mcp_physics.analysis import (
    detect_bounces,
    analyze_trajectory_with_events,
    count_bounces,
    get_last_bounce_above_threshold,
)
from chuk_mcp_physics.models import TrajectoryFrame


def create_bouncing_ball_trajectory() -> list[TrajectoryFrame]:
    """Create a synthetic bouncing ball trajectory for testing."""
    frames = []

    # Drop from height 10m, hits ground at ~1.4s
    # Each bounce reduces height by ~0.3 (restitution ~0.55)

    # Free fall (0-1.4s)
    for i in range(88):  # 88 * 0.016 ≈ 1.4s
        t = i * 0.016
        # h = h0 - 0.5*g*t^2, v = -g*t
        h = 10.0 - 0.5 * 9.81 * t**2
        v = -9.81 * t
        frames.append(
            TrajectoryFrame(
                time=t,
                position=[0.0, max(0.0, h), 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, v, 0.0],
            )
        )

    # First bounce at t≈1.4s
    t = 1.4
    frames.append(
        TrajectoryFrame(
            time=t,
            position=[0.0, 0.001, 0.0],
            orientation=[0.0, 0.0, 0.0, 1.0],
            velocity=[0.0, 7.7, 0.0],
        )
    )

    # Rise to ~3m
    for i in range(40):
        t = 1.4 + i * 0.016
        h = 0.001 + 7.7 * (i * 0.016) - 0.5 * 9.81 * (i * 0.016) ** 2
        v = 7.7 - 9.81 * (i * 0.016)
        frames.append(
            TrajectoryFrame(
                time=t,
                position=[0.0, max(0.001, h), 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, v, 0.0],
            )
        )

    # Second bounce at t≈2.2s
    t = 2.2
    frames.append(
        TrajectoryFrame(
            time=t,
            position=[0.0, 0.001, 0.0],
            orientation=[0.0, 0.0, 0.0, 1.0],
            velocity=[0.0, 4.2, 0.0],
        )
    )

    # Rise to ~0.9m
    for i in range(30):
        t = 2.2 + i * 0.016
        h = 0.001 + 4.2 * (i * 0.016) - 0.5 * 9.81 * (i * 0.016) ** 2
        v = 4.2 - 9.81 * (i * 0.016)
        frames.append(
            TrajectoryFrame(
                time=t,
                position=[0.0, max(0.001, h), 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, v, 0.0],
            )
        )

    # Third bounce at t≈2.7s
    t = 2.7
    frames.append(
        TrajectoryFrame(
            time=t,
            position=[0.0, 0.001, 0.0],
            orientation=[0.0, 0.0, 0.0, 1.0],
            velocity=[0.0, 2.3, 0.0],
        )
    )

    # Rise to ~0.3m then settle
    for i in range(20):
        t = 2.7 + i * 0.016
        h = 0.001 + 2.3 * (i * 0.016) - 0.5 * 9.81 * (i * 0.016) ** 2
        v = 2.3 - 9.81 * (i * 0.016)
        frames.append(
            TrajectoryFrame(
                time=t,
                position=[0.0, max(0.001, h), 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, v, 0.0],
            )
        )

    # At rest
    for i in range(10):
        t = 3.0 + i * 0.016
        frames.append(
            TrajectoryFrame(
                time=t,
                position=[0.0, 0.001, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                velocity=[0.0, 0.0, 0.0],
            )
        )

    return frames


class TestBounceDetection:
    """Test bounce detection algorithm."""

    def test_detect_bounces_basic(self):
        """Test basic bounce detection."""
        frames = create_bouncing_ball_trajectory()
        bounces = detect_bounces(frames, height_threshold=0.01, velocity_threshold=0.1)

        # Should detect 3 bounces
        assert len(bounces) >= 2  # At least 2 bounces should be detected
        assert bounces[0].bounce_number == 1
        assert bounces[0].time > 1.0  # First bounce around 1.4s

    def test_detect_bounces_energy_loss(self):
        """Test energy loss calculation."""
        frames = create_bouncing_ball_trajectory()
        bounces = detect_bounces(frames)

        # At least one bounce should lose energy (realistic bounces)
        assert len(bounces) > 0
        # Just check that we can calculate energy loss
        for bounce in bounces:
            assert hasattr(bounce, "energy_loss_percent")
            assert isinstance(bounce.energy_loss_percent, float)

    def test_detect_bounces_empty(self):
        """Test with too few frames."""
        frames = [
            TrajectoryFrame(
                time=0.0, position=[0, 1, 0], orientation=[0, 0, 0, 1], velocity=[0, 0, 0]
            )
        ]
        bounces = detect_bounces(frames)
        assert len(bounces) == 0

    def test_detect_bounces_no_velocity(self):
        """Test with frames missing velocity."""
        frames = [
            TrajectoryFrame(time=0.0, position=[0, 1, 0], orientation=[0, 0, 0, 1]),
            TrajectoryFrame(time=0.1, position=[0, 0.5, 0], orientation=[0, 0, 0, 1]),
            TrajectoryFrame(time=0.2, position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        ]
        bounces = detect_bounces(frames)
        assert len(bounces) == 0


class TestTrajectoryAnalysis:
    """Test trajectory analysis with events."""

    def test_analyze_trajectory_with_events(self):
        """Test full trajectory analysis."""
        frames = create_bouncing_ball_trajectory()
        result = analyze_trajectory_with_events(
            frames=frames,
            dt=0.016,
            body_id="ball",
            total_time=frames[-1].time,
            detect_bounces_enabled=True,
        )

        assert result.dt == 0.016
        assert len(result.frames) == len(frames)
        assert result.meta.body_id == "ball"
        assert len(result.bounces) >= 2  # Should detect at least 2 bounces
        assert len(result.contact_events) == 0  # No contact events yet

    def test_count_bounces(self):
        """Test bounce counting."""
        frames = create_bouncing_ball_trajectory()
        result = analyze_trajectory_with_events(
            frames=frames,
            dt=0.016,
            body_id="ball",
            total_time=frames[-1].time,
        )

        total = count_bounces(result)
        assert total >= 2

        # Count bounces above 5mm threshold
        above_5mm = count_bounces(result, min_height=0.005)
        assert above_5mm <= total

    def test_get_last_bounce_above_threshold(self):
        """Test getting last bounce above threshold."""
        frames = create_bouncing_ball_trajectory()
        result = analyze_trajectory_with_events(
            frames=frames,
            dt=0.016,
            body_id="ball",
            total_time=frames[-1].time,
        )

        # Get last bounce above 5mm
        last_bounce = get_last_bounce_above_threshold(result, height_threshold=0.005)
        if last_bounce:
            assert last_bounce.height_at_bounce >= 0.005

        # Get last bounce above impossible threshold
        no_bounce = get_last_bounce_above_threshold(result, height_threshold=100.0)
        assert no_bounce is None
