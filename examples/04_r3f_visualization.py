#!/usr/bin/env python3
"""Example 4: React Three Fiber Visualization Data

This example shows how to generate trajectory data that's ready to use
in React Three Fiber for 3D animations.
"""

import asyncio
import json
from chuk_mcp_physics.providers.analytic import AnalyticProvider
from chuk_mcp_physics.models import ProjectileMotionRequest


async def main():
    """Generate R3F-compatible animation data."""

    provider = AnalyticProvider()

    print("=" * 60)
    print("REACT THREE FIBER VISUALIZATION DATA")
    print("=" * 60)
    print()

    # Example 1: Basketball shot trajectory
    print("1. Basketball Shot - Generating R3F Trajectory")
    print("-" * 60)

    request = ProjectileMotionRequest(
        initial_velocity=7.0, angle_degrees=52.0, initial_height=2.0, gravity=9.81
    )

    result = await provider.calculate_projectile_motion(request)

    print(f"Generated {len(result.trajectory_points)} trajectory points")
    print(f"Total flight time: {result.time_of_flight:.2f} seconds")
    print(f"Maximum height: {result.max_height:.2f} m")
    print(f"Range: {result.range:.2f} m")
    print()

    # Convert to R3F format (3D coordinates, add z=0)
    r3f_trajectory = []
    dt = result.time_of_flight / len(result.trajectory_points)

    for i, (x, y) in enumerate(result.trajectory_points):
        r3f_trajectory.append({"time": i * dt, "position": [x, y, 0.0], "index": i})

    # Show sample
    print("Sample R3F trajectory data (first 5 points):")
    print(json.dumps(r3f_trajectory[:5], indent=2))
    print()

    # Generate React Three Fiber component code
    print("React Three Fiber Component Example:")
    print("-" * 60)

    r3f_code = f"""
import {{ useRef, useState, useEffect }} from 'react';
import {{ useFrame }} from '@react-three/fiber';
import * as THREE from 'three';

// Trajectory data from physics MCP server
const trajectoryData = {json.dumps(r3f_trajectory[:10], indent=2)}...
// ({len(result.trajectory_points)} total points)

function Basketball() {{
  const ballRef = useRef();
  const [trajectoryIndex, setTrajectoryIndex] = useState(0);
  const animationSpeed = 1.0; // 1.0 = real-time, 0.5 = half speed

  useFrame((state, delta) => {{
    if (!ballRef.current || trajectoryIndex >= trajectoryData.length - 1) return;

    // Advance through trajectory based on time
    const newIndex = Math.min(
      trajectoryData.length - 1,
      trajectoryIndex + (delta * animationSpeed * {len(result.trajectory_points) / result.time_of_flight:.1f})
    );
    setTrajectoryIndex(newIndex);

    // Interpolate between frames for smooth motion
    const currentFrame = trajectoryData[Math.floor(newIndex)];
    const nextFrame = trajectoryData[Math.ceil(newIndex)];
    const t = newIndex % 1;

    ballRef.current.position.lerpVectors(
      new THREE.Vector3(...currentFrame.position),
      new THREE.Vector3(...nextFrame.position),
      t
    );
  }});

  return (
    <mesh ref={{ballRef}} castShadow>
      <sphereGeometry args={{[0.12, 32, 32]}} /> {{/* 12cm basketball */}}
      <meshStandardMaterial color="orange" roughness={{0.8}} />
    </mesh>
  );
}}

function BasketballCourt() {{
  return (
    <group>
      {{/* Ground */}}
      <mesh rotation={{[-Math.PI / 2, 0, 0]}} receiveShadow>
        <planeGeometry args={{[20, 20]}} />
        <meshStandardMaterial color="#8B4513" />
      </mesh>

      {{/* Basketball hoop at (4.6, 3.05, 0) */}}
      <mesh position={{[4.6, 3.05, 0]}}>
        <torusGeometry args={{[0.23, 0.02, 16, 32]}} />
        <meshStandardMaterial color="#ff4500" />
      </mesh>

      {{/* Backboard */}}
      <mesh position={{[4.6, 3.5, 0]}}>
        <boxGeometry args={{[1.8, 1.05, 0.05]}} />
        <meshStandardMaterial color="white" transparent opacity={{0.3}} />
      </mesh>
    </group>
  );
}}

export default function BasketballScene() {{
  return (
    <>
      <ambientLight intensity={{0.5}} />
      <directionalLight position={{[10, 10, 5]}} castShadow />
      <Basketball />
      <BasketballCourt />
    </>
  );
}}
"""

    print(r3f_code)
    print()

    # Example 2: Multiple trajectories for comparison
    print("2. Multiple Trajectories - Different Angles")
    print("-" * 60)

    angles = [30, 45, 60]
    all_trajectories = {}

    for angle in angles:
        request = ProjectileMotionRequest(
            initial_velocity=20.0, angle_degrees=float(angle), initial_height=0.0, gravity=9.81
        )
        result = await provider.calculate_projectile_motion(request)

        trajectory_3d = []
        dt = result.time_of_flight / len(result.trajectory_points)

        for i, (x, y) in enumerate(result.trajectory_points):
            trajectory_3d.append({"t": round(i * dt, 3), "pos": [round(x, 2), round(y, 2), 0.0]})

        all_trajectories[f"angle_{angle}"] = {
            "angle": angle,
            "max_height": round(result.max_height, 2),
            "range": round(result.range, 2),
            "flight_time": round(result.time_of_flight, 2),
            "trajectory": trajectory_3d[:20],  # First 20 points as sample
        }

    print("Trajectory comparison data (JSON):")
    print(json.dumps(all_trajectories, indent=2))
    print()

    # Example 3: Trail visualization data
    print("3. Trail Visualization - Path Markers")
    print("-" * 60)

    request = ProjectileMotionRequest(
        initial_velocity=25.0, angle_degrees=40.0, initial_height=1.5, gravity=9.81
    )
    result = await provider.calculate_projectile_motion(request)

    # Create markers every 0.5 seconds
    marker_interval = 0.5  # seconds
    dt = result.time_of_flight / len(result.trajectory_points)

    trail_markers = []
    for i, (x, y) in enumerate(result.trajectory_points):
        t = i * dt
        if i % int(marker_interval / dt) == 0:
            trail_markers.append(
                {
                    "time": round(t, 2),
                    "position": [round(x, 2), round(y, 2), 0.0],
                    "label": f"t={t:.1f}s",
                }
            )

    print(f"Generated {len(trail_markers)} trail markers:")
    print(json.dumps(trail_markers, indent=2))
    print()

    print("R3F Trail Component:")
    print("-" * 60)
    trail_code = """
function TrajectoryTrail({ markers }) {
  return (
    <group>
      {markers.map((marker, i) => (
        <group key={i} position={marker.position}>
          {/* Trail sphere */}
          <mesh>
            <sphereGeometry args={[0.1, 16, 16]} />
            <meshStandardMaterial
              color="cyan"
              transparent
              opacity={0.6}
            />
          </mesh>

          {/* Time label */}
          <Html center distanceFactor={10}>
            <div style={{
              color: 'white',
              fontSize: '12px',
              background: 'rgba(0,0,0,0.5)',
              padding: '2px 4px',
              borderRadius: '3px'
            }}>
              {marker.label}
            </div>
          </Html>
        </group>
      ))}
    </group>
  );
}
"""
    print(trail_code)
    print()

    # Save to file for easy import
    output_file = "basketball_trajectory.json"
    with open(output_file, "w") as f:
        json.dump(
            {
                "metadata": {
                    "initial_velocity": request.initial_velocity,
                    "angle": request.angle_degrees,
                    "max_height": result.max_height,
                    "range": result.range,
                    "flight_time": result.time_of_flight,
                },
                "trajectory": r3f_trajectory,
                "trail_markers": trail_markers,
            },
            f,
            indent=2,
        )

    print(f"✓ Saved complete trajectory data to: {output_file}")
    print()
    print("Usage in React:")
    print("  import trajectoryData from './basketball_trajectory.json';")
    print()
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
