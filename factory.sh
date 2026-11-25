#!/bin/bash
# Helper for running and testing the Smart Mini Factory simulation.
#
# From Windows PowerShell:
#   wsl -d ubuntu26 -- /home/ros_lyrical/robot_arm_ws/factory.sh <command> [args]
# Inside Ubuntu:
#   ~/robot_arm_ws/factory.sh <command> [args]

source /opt/ros/lyrical/setup.bash
source "$(dirname "$(readlink -f "$0")")/install/setup.bash"

call() { ros2 service call "$@" | grep -oE "(success=[A-Za-z]+|message='[^']*')" | tr '\n' ' '; echo; }

# the nodes declare these as floating point: "45" must be sent as "45.0"
flt() { case "$1" in *.*) echo "$1" ;; *) echo "$1.0" ;; esac; }

JAM_SDF='<sdf version="1.9"><model name="jam_block"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.02 0.04 0.03</size></box></geometry></collision><visual name="v"><geometry><box><size>0.02 0.04 0.03</size></box></geometry><material><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model></sdf>'

case "$1" in
  start)            # whole cell with the Gazebo window and RViz; extra args go to the launch
    shift
    exec ros2 launch factory_bringup factory.launch.py "$@" ;;
  status)           # live one-line status, once a second (Ctrl+C to stop)
    exec python3 - <<'EOF'
import rclpy
from factory_interfaces.msg import CellStatus
rclpy.init(); n = rclpy.create_node('status_view'); last = {}
n.create_subscription(CellStatus, '/cell/status', lambda m: last.__setitem__('m', m), 10)
import time
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=1.0)
    m = last.get('m')
    if m:
        bins = ' '.join(f'B{i+1}:{c}' for i, c in enumerate(m.bin_counts))
        print(f'{m.state:9s} light={m.tower:6s} in={m.entered:3d} sorted={m.sorted:3d} '
              f'rejected={m.rejected:3d} [{bins}] {m.items_per_min:.0f}/min'
              + (f'  FAULT: {m.fault}' if m.fault else ''), flush=True)
    time.sleep(1.0)
EOF
    ;;
  camera)           # what the vision sees, with labels
    exec ros2 run rqt_image_view rqt_image_view /vision/debug ;;
  estop)            call /estop/set std_srvs/srv/SetBool "{data: true}" ;;
  release)          call /estop/set std_srvs/srv/SetBool "{data: false}" ;;
  reset)            call /cell/reset std_srvs/srv/Trigger ;;
  pause)            call /cell/stop std_srvs/srv/Trigger ;;
  resume)           call /cell/start std_srvs/srv/Trigger ;;
  jam)              # block the IR2 beam with a fixed obstacle
    call /world/factory/create ros_gz_interfaces/srv/SpawnEntity \
      "{entity_factory: {name: jam_block, sdf: '$JAM_SDF', pose: {position: {x: 0.54, y: 0.38, z: 0.066}}}}" ;;
  unjam)            call /world/factory/remove ros_gz_interfaces/srv/DeleteEntity "{entity: {name: jam_block, type: 2}}" ;;
  pc-crash)         call /cell/pc_crash std_srvs/srv/SetBool "{data: true}" ;;
  pc-ok)            call /cell/pc_crash std_srvs/srv/SetBool "{data: false}" ;;
  speed)            ros2 param set /supervisor belt_speed_mm_s "$(flt "${2:?give mm/s, e.g. 45}")" ;;
  feed)             ros2 param set /spawner feed_rate_per_min "$(flt "${2:?give products/min, e.g. 15}")" ;;
  defects)          ros2 param set /spawner defect_fraction "$(flt "${2:?give 0..1, e.g. 0.5}")" ;;
  report)           ros2 run factory_control report ;;
  calibrate)        # redo the camera calibration (needs the sim running)
    exec ros2 run factory_control calibrate_node --ros-args -p use_sim_time:=true ;;
  *)
    cat <<'EOF'
usage: factory.sh <command>

  start [gui:=false rviz:=false max_products:=N seed:=N]   run the whole cell
  status            live state, light, counts
  camera            vision debug image
  estop | release | reset          E-stop test
  jam | unjam       block / clear the IR2 beam
  pc-crash | pc-ok  stop / resume the computer heartbeat (watchdog test)
  pause | resume    stop / start the line
  speed MM_S        belt speed (default 30)
  feed PER_MIN      products per minute (default 10)
  defects FRACTION  share of defective products (default 0.12)
  report            results from the database
  calibrate         redo the camera calibration
EOF
    ;;
esac
