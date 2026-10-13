import sys

from tools.tuning.hitl_bridge.bridge import main
from tools.tuning.hitl_bridge.process_priority import raise_bridge_priority

print(f"bridge: {raise_bridge_priority()}", flush=True)     # the process entry only, not in-process callers
sys.exit(main())
