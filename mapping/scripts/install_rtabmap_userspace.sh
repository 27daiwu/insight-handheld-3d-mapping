#!/usr/bin/env bash
# Install RTAB-Map + MCAP storage for ROS 2 Humble WITHOUT root: download the .deb files apt would install and
# extract them into a private prefix.  Does not touch the system, drivers or CUDA.
# (With sudo you can simply: sudo apt install ros-humble-rtabmap-ros ros-humble-rosbag2-storage-mcap)
set -euo pipefail
PREFIX="${INSIGHT_ROS_EXTRA:-$HOME/.local/opt/ros-extra}"
mkdir -p "$PREFIX/debs"; cd "$PREFIX/debs"
PK=$(apt-get -s install ros-humble-rtabmap-slam ros-humble-rtabmap-odom ros-humble-rtabmap-util ros-humble-rtabmap-sync \
      ros-humble-rtabmap-launch ros-humble-rtabmap-msgs ros-humble-rtabmap-conversions ros-humble-rosbag2-storage-mcap 2>&1 \
      | awk '/^Inst/{print $2}' | grep -v -E "viz|rviz|-dev$" || true)
[ -n "$PK" ] && apt-get download $PK
for d in *.deb; do dpkg-deb -x "$d" "$PREFIX/root"; done
cat > "$PREFIX/env.sh" <<'EOS'
R="${INSIGHT_ROS_EXTRA:-$HOME/.local/opt/ros-extra}/root"
export AMENT_PREFIX_PATH=$R/opt/ros/humble:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=$R/opt/ros/humble:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$R/opt/ros/humble/lib:$R/opt/ros/humble/lib/x86_64-linux-gnu:$R/opt/ros/humble/opt/gtsam/lib:$R/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PATH=$R/opt/ros/humble/bin:$PATH
export PYTHONPATH=$R/opt/ros/humble/local/lib/python3.10/dist-packages:$R/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
EOS
echo "done; use: source mapping/scripts/env.sh"
