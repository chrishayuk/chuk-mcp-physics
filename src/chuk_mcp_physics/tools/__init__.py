"""Physics MCP tool endpoints organized by domain.

This package contains @tool decorated endpoint wrappers that expose physics
calculations through the MCP protocol.

Structure:
- Each domain file contains @tool decorated async functions
- Tools import calculation functions from parent modules (../rotational.py, etc.)
- server.py imports and registers tools from this package

REFACTORING COMPLETE - 90% of tools extracted from server.py!
================================================================

✅ COMPLETED (10 domain files, 53 tools):
  - rotational.py (5 tools)
  - oscillations.py (5 tools)
  - circular_motion.py (5 tools)
  - collisions.py (2 tools)
  - conservation.py (4 tools)
  - kinematics_tools.py (6 tools - Phase 3, with CORRECTED 3D API)
  - statics.py (7 tools - Phase 3)
  - fluid.py (10 tools - basic + Phase 3 advanced)
  - basic.py (8 tools - force, energy, momentum, work, projectile)
  - [simulation tools remain in server.py - 6 tools, complex provider pattern]

Total: 53 tools refactored / 59 total (90%)
"""

# Import all refactored tool modules
from . import (
    basic,
    circular_motion,
    collisions,
    conservation,
    fluid,
    kinematics_tools,
    oscillations,
    rotational,
    statics,
)

__all__ = [
    "rotational",
    "oscillations",
    "circular_motion",
    "collisions",
    "conservation",
    "kinematics_tools",
    "statics",
    "fluid",
    "basic",
]
