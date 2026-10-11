"""Navigation module - Final-approach navigation and path following.

This module owns ALL navigation-related functionality:
- Approach geometry for a selected POI
- Trajectory/path following
- Geo-reference math (via navigation.geo submodule)
- Terrain utilities (via navigation.geo submodule)
- Navigation algorithms (L1 control, PID)
- Mission planning

Generic target/setpoint and standard control terms retain their engineering
meaning. Approach geometry and
approach accuracy does not establish a mission outcome (e.g. docking, cargo receipt, coverage).
"""
