# Uno firmware and the serial bridge

Phase 2 of the Smart Mini Factory: an Arduino Uno takes over the real-time and
safety layer from the simulated belt-and-I/O node (`conveyor_io_node`). The
supervisor, vision and arm code do not change, because the bridge publishes the
same ROS topics.

```
supervisor ──ROS topics──> uno_bridge_node ──USB serial 115200──> Uno (uno_factory)
(laptop / PC)                /io/reset, /conveyor/cmd, ...           belt, relay, servos, sensors
```

## Layout

| Path | What |
|---|---|
| `uno_factory/core.cpp, core.h` | Protocol and every safety rule. No Arduino calls, so it also runs on the PC. |
| `uno_factory/uno_factory.ino` | Hardware layer: pins, step timer, PCA9685, ADC. |
| `uno_factory/config.h` | Pins' behaviour, limits, rest pose. **`BENCH_MODE` is 1** (see below). |
| `host/test_core.cpp` | 91 checks of the real `core.cpp` on the PC. |
| `host/uno_sim.cpp` | The same core behind a virtual serial port (a simulated Uno). |
| `host/bridge_check.py` | Simulated Uno + real bridge + real supervisor: 17 operator-style checks. |
| `../src/factory_control/factory_control/uno_bridge_node.py` | The ROS bridge. |
| `../src/factory_control/factory_control/uno_protocol.py` | Line format and parser (documented at the top). |

## Run the PC tests (no hardware)

```bash
cd ~/robot_arm_ws/firmware/host
make test                      # 91 firmware checks (address and undefined-behaviour sanitisers on)
python3 bridge_check.py        # needs the workspace built and sourced
```

## Safety behaviour (what the code does, checked on the PC only)

- The relay closes only if: the E-stop chain reads closed (LOW on D7), heartbeats
  are fresh, a servo pose was sent first, and the 12 V reading is above 10.5 V.
- E-stop open (or a broken wire) opens the relay in the same loop pass and stops the belt.
- No heartbeat for 2 s: the belt stops and the arm moves to the rest pose over 3 s,
  then the relay opens. The Uno's own watchdog resets it if the loop hangs.
- Servo-rail current above 6 A for 300 ms: the gripper opens and the belt stops;
  if the current is still high 1 s later the relay opens.
- A fault never clears by itself. `RST` (via `/io/reset` or `/cell/reset`) clears it,
  and only with the E-stop released.
- The E-stop also cuts the relay coil in hardware (see the wiring simulator), so it
  works even if this firmware hangs.

## Before you connect real motors

1. **`BENCH_MODE`**: set to 0 in `config.h` once the 12 V supply and the ACS712-20A
   are wired. While it is 1, the current and voltage checks are skipped.
2. **`REST_POSE` and `GRIPPER_OPEN_US`** are placeholders. Measure them on your arm.
3. **`IR_BLOCKED_LEVEL`**: check which level your IR pair gives when blocked.
4. Check `STEPS_PER_MM` against your roller, and set the TMC2209 current first.
5. The Uno bootloader must support the watchdog (Optiboot, which ships on genuine
   Uno R3 boards). An old bootloader can hang after a watchdog reset.

## Not tested on a real board

Everything above was tested on a PC with a simulated Uno. These need the real
board and will probably need adjusting: the 20 kHz step interrupt and the step
rate it gives, the PCA9685 register setup (50 Hz), the ADC scaling for the 12 V
divider and the ACS712, the I2C timeout, the start-up pin states, and the IR
inputs, which are read in the main loop (with a 10 ms debounce) rather than by
interrupt. The firmware compiles for the Uno (about 12 KB of 32 KB flash, 0.8 KB of 2 KB RAM).

## Upload and first bench test

1. Install the Arduino IDE, open `firmware/uno_factory/uno_factory.ino` (keep the
   three other files in that folder), choose *Arduino Uno*, upload.
2. Leave `BENCH_MODE` at 1 and the 12 V supply and motors disconnected.
3. Pass the Uno from Windows into WSL (see below), then run
   `ros2 run factory_control uno_bridge_node --ros-args -p port:=/dev/ttyACM0`
   (a CH340 clone appears as `/dev/ttyUSB0`) and watch `ros2 topic echo /uno/state` and `/ir1`.
4. The Uno will refuse the relay until it has a servo pose: pass
   `-p send_home:=true` (with `home_us` set to your measured rest pose) or publish
   six pulse widths to `/uno/servos_us`.

## Laptop setup: USB devices into WSL

There is no Raspberry Pi: the laptop runs ROS 2 inside WSL. WSL does not see USB
devices by itself, so Windows passes them in with `usbipd-win`. The WSL kernel on
this PC (6.6.87.2) ships the needed modules (`vhci-hcd`, `cdc-acm`, `ch341`,
`uvcvideo`). None of the steps below has been tried with a real device yet.

In PowerShell, install once, then list devices:

```powershell
winget install usbipd
usbipd list
```

For each device (use its BUSID from the list). `bind` needs an administrator
PowerShell and is only needed once; `attach` is needed after every replug:

```powershell
usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```

In Ubuntu, if `/dev/ttyACM0` (or `/dev/ttyUSB0`, `/dev/video0`) does not appear,
load the module: `sudo modprobe cdc-acm` (or `ch341`, `uvcvideo`). For permission
errors on the serial port: `sudo usermod -aG dialout $USER`, then restart WSL.

Open points:
- A webcam over usbipd may be too slow or unreliable at 720p. If so, the options
  are a lower resolution, reading the camera on the Windows side and sending the
  frames to ROS, or installing Ubuntu directly on the laptop.
- The vision node reads `/camera/image_raw`. A real camera needs a camera driver
  node to publish it (the simulation gets it from Gazebo).
- The arm is not driven on real hardware yet: `arm_node` sends trajectories to the
  Gazebo controllers, and nothing converts them into `/uno/servos_us` pulses.

