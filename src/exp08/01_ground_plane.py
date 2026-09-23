"""EXP-08 snippet 1 — paste into Window > Script Editor, then Run.

Official Basic Usage Tutorial, Extensions tab.
https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html
"""

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import GroundPlane

stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
