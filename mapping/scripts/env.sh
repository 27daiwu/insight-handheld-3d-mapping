# Source this before running insight-mapping (bag reading + RTAB-Map):  source mapping/scripts/env.sh
# ROS 2 Humble + optional user-space overlay holding RTAB-Map/MCAP extracted from .deb files (no root needed).
source /opt/ros/humble/setup.bash
OVERLAY="${INSIGHT_ROS_EXTRA:-$HOME/.local/opt/ros-extra}"
[ -f "$OVERLAY/env.sh" ] && source "$OVERLAY/env.sh"
