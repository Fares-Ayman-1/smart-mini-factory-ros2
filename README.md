# Smart Mini Factory — ROS 2 simulation

Simulation of the Smart Mini Factory sorting cell (design docs live in `D:\robot_arm`):
a 600 mm belt with two IR beams, a camera in a lit hood, a 6-joint arm and five
bins, run by ROS 2 nodes. The microcontroller side (belt, IR, E-stop, relay,
watchdog) is emulated by `conveyor_io_node`; Phase 2 swaps in a serial bridge
to an Arduino Uno with the same ROS topics.

- ROS 2 Lyrical, Gazebo 10 (gz sim), WSL `ubuntu26`
- World frame = baseplate front-left corner; x along the belt, y to the back, z up; SI units.
- All positions come from `factory_control/geometry.py`; the Gazebo world is
  generated from it (`src/factory_gazebo/scripts/generate_world.py`).

## Setup (once)

```bash
sudo apt install ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
                 ros-lyrical-gz-ros2-control ros-lyrical-joint-state-publisher-gui
cd ~/robot_arm_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```
## Run

```bash
# 1. Calibrate the camera (writes src/factory_bringup/config/calibration.yaml)
ros2 launch factory_gazebo sim.launch.py with_robot:=false
ros2 run factory_control calibrate_node --ros-args -p use_sim_time:=true     # 2nd terminal

# 2. The whole cell (Gazebo GUI + RViz)
ros2 launch factory_bringup factory.launch.py
ros2 run rqt_image_view rqt_image_view /vision/debug                          # what the camera sees
ros2 topic echo /cell/status                                                  # state, counts, rate

# 3. Results (SQLite at ~/robot_arm_ws/data/factory.db)
ros2 run factory_control report
```

Headless: `ros2 launch factory_bringup factory.launch.py gui:=false rviz:=false`.

### Try the safety logic

```bash
ros2 service call /estop/set std_srvs/srv/SetBool "{data: true}"    # press E-stop -> ESTOP, red
ros2 service call /estop/set std_srvs/srv/SetBool "{data: false}"   # release
ros2 service call /cell/reset std_srvs/srv/Trigger                  # acknowledge, back to RUNNING
ros2 service call /cell/pi_crash std_srvs/srv/SetBool "{data: true}" # stop heartbeats -> watchdog stops the belt
```

### Bench tools

```bash
ros2 launch factory_gazebo sim.launch.py gui:=false
ros2 run factory_control arm_node --ros-args -p use_sim_time:=true
ros2 run factory_control grasp_trial --ros-args -p trials:=20 -p use_sim_time:=true
ros2 launch factory_description display.launch.py                   # arm + joint sliders in RViz
colcon test --packages-select factory_control && colcon test-result --verbose
```

## Packages

| Package | Contents |
|---|---|
| `factory_interfaces` | `Detection`, `CellStatus` messages, `PickPlace` service |
| `factory_description` | Arm URDF/xacro, ros2_control tags, RViz config |
| `factory_gazebo` | Generated cell world (belt, hood, camera, IR beams, bins), sim launch, gz bridge |
| `factory_bringup` | Controller config, cell config (sorting table, speeds), calibration, full launch |
| `factory_control` | IK, vision, and the nodes: arm, conveyor I/O, spawner, vision, calibration, supervisor, logger, pose relay |
| `factory_gz_plugins` | `GraspAttach` Gazebo system (sim-only grasp assist) |

## Nodes and topics

| Node | Role | Key interfaces |
|---|---|---|
| `conveyor_io` | Microcontroller emulation: belt ramp, IR debounce, E-stop, relay, 2 s watchdog | `/conveyor/cmd`, `/ir1`, `/ir2`, `/estop`, `/relay/state`, `/estop/set` |
| `arm` | Pick-and-place sequencer on closed-form IK | `/arm/pick_place`, `/arm/home`, `/arm/halt` |
| `vision` | HSV segmentation, colour/shape/defect, homography + parallax correction | `/vision/detection`, `/vision/debug` |
| `supervisor` | Cell state machine, sorting table, faults, heartbeat | `/cell/status`, `/cell/events`, `/cell/start|stop|reset` |
| `spawner` | Feeds random products, removes finished ones | `/products/spawned` |
| `item_logger` | SQLite row per item, scored against ground truth | `~/robot_arm_ws/data/factory.db` |

## Design notes found while building the sim

These are observations from this simulation, not verified facts about the real
machine. Several depend on dimensions I assumed (marked *assumed*), so check them
against your real parts before cutting or printing anything.

- **Hood arm-side wall vs. arm sweep.** Not tried: folding the arm up to pass the wall. The wall at y = 300 mm is only 60 mm from
  the arm's base axis; swinging between the bins and the belt, the arm sweeps
  through it (~225 mm radius at ~150–165°). The sim leaves that side open.
- **Grasp height** (*assumed* 40 mm finger pads). With 40 mm finger pads centred on the TCP, gripping at the
  product centre (z = 65 mm) drives the finger tips into the belt; the arm grips
  with the tips 3 mm above the belt instead (TCP z = 73 mm).
- **Pick stop** (*assumed* the product's front edge trips IR2). IR2 trips on the product's front edge, so the belt runs ~1.1 s
  longer to centre the product in the pick zone (`stop_delay_s`).
- **Camera parallax.** At 200 mm height a 30 mm part's top face appears ~18 mm
  outward at the pick zone; the vision corrects for it.
- **Camera position.** At the PDF's x = 440 mm the 283 mm view ends at 581 mm and
  clips the top face of a product at the 560 mm pick zone. The sim mounts it at
  x = 470 mm, long image axis along the belt.
- **IR2 vs. the gripper** (*assumed* finger and beam geometry). With the fingers open along the belt, a finger crosses
  the IR2 beam (x = 540 mm) while picking at 560 mm, so jam detection only runs
  while the belt is moving.
- **Wrist roll vs. the rails** (*assumed* 22 mm wide pads, 54 mm opening). Open finger pads rolled past ~15° (with a product a few mm off centre) hit the lane
  rails (30 mm from the centreline); roll is capped and cylinders get none.
- **Grasping in simulation.** Friction-only grasps of 30 mm parts slip in Gazebo,
  so a sim-only plugin (`factory_gz_plugins/GraspAttach`) locks the product to
  the gripper once the fingers have closed on it — only if it really sits
  between the pads (±12 mm). Turn it off with the arm's `grasp_assist:=false`.

## Known limits of the simulation

- Fault coverage: E-stop, product jam and a lost supervisor heartbeat are exercised.
  Servo stall, belt stall and 12 V brown-out are **not** simulated in Gazebo (the
  Uno firmware handles servo overcurrent and brown-out; those are tested on the PC only).
- The Gazebo sim does not park the arm when the heartbeat is lost. The Uno firmware does.
- Joint speed limits in the arm model are estimates, not datasheet values.
- The 50-product run (50/50 picked, classified and placed; 5/5 defects caught) was
  a single seeded run whose database was not kept. Rerun it with
  `factory.sh start gui:=false rviz:=false max_products:=50 seed:=42`.
- Hardware phase: see `firmware/README.md` (Uno firmware and serial bridge, tested
  only against a simulated Uno).
