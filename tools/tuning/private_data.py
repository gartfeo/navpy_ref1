"""Where the HITL tools find the aircraft's own data, which stays out of the repository.

The aero models, the aircraft's parameter files and the launcher's measurements belong to the owner and are
not published. They live in a private folder: NAVPY_HITL_TOOLS_DIR when set, else navpy-hitl-tools under
%LOCALAPPDATA% (Windows) or the home folder. The JSBSim models are passed by path (--model-dir); the files the
runner and the bridge read by default are in its hitl subfolder (HITL_DATA_DIR_NAME):

- hitl_aircraft.parm (AIRCRAFT_PARAMS_NAME): the aircraft's airspeeds and takeoff settings, loaded with
  --bridge after hitl_bridge.parm;
- hitl_launch.parm (LAUNCH_PARAMS_NAME): launch detection and motor delay, loaded with --bridge after the
  starting parameters (--params-file);
- launch.json (LAUNCH_PROFILE_NAME): the catapult rail and the rest pose on the skids, read by the bridge
  (hitl_bridge/launch_profile.py).

Each tool also takes the folder or the file as an argument (sitl_driver/hitl --hitl-data, bridge
--launch-file), so a WSL process can be pointed at the Windows folder under /mnt/c.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping, Optional

PRIVATE_DIR_ENV = "NAVPY_HITL_TOOLS_DIR"
PRIVATE_DIR_NAME = "navpy-hitl-tools"
LOCAL_APPDATA_ENV = "LOCALAPPDATA"
HITL_DATA_DIR_NAME = "hitl"
AIRCRAFT_PARAMS_NAME = "hitl_aircraft.parm"
LAUNCH_PARAMS_NAME = "hitl_launch.parm"
LAUNCH_PROFILE_NAME = "launch.json"


def private_dir(environ: Optional[Mapping[str, str]] = None) -> Path:
    """The private folder: NAVPY_HITL_TOOLS_DIR, else navpy-hitl-tools under LOCALAPPDATA or the home folder."""
    env = os.environ if environ is None else environ
    if env.get(PRIVATE_DIR_ENV):
        return Path(env[PRIVATE_DIR_ENV]).expanduser()
    base = Path(env[LOCAL_APPDATA_ENV]) if env.get(LOCAL_APPDATA_ENV) else Path.home()
    return base / PRIVATE_DIR_NAME


def hitl_data_dir(environ: Optional[Mapping[str, str]] = None) -> Path:
    """The folder of the aircraft's HITL parameter files and launch profile."""
    return private_dir(environ) / HITL_DATA_DIR_NAME
