// Settings for the Uno build. Pins match the wiring simulator.
#pragma once
// 1 = desk test without the 12 V supply and current sensor (those checks are
// skipped). Set to 0 before the relay switches real motors.
#define BENCH_MODE 1
#define IR_BLOCKED_LEVEL HIGH   // level on D2/D3 when the beam is blocked: check your sensor

#define STEPS_PER_MM 33.95f     // 200 * 16 / (pi * 30 mm)
#define BELT_MAX_DMM 600        // 60.0 mm/s, in tenths of mm/s
#define BELT_ACCEL_DMM_S 2000   // 200 mm/s^2
#define HEARTBEAT_MS 2000
#define PARK_MS 3000            // slow move to the rest pose after a lost heartbeat
#define OVERCURRENT_MA 6000
#define OVERCURRENT_MS 300
#define OVERCURRENT_CUT_MS 1000 // still high this long after the fault: open the relay
#define BROWNOUT_MV 10500
#define BROWNOUT_MS 200
#define DEBOUNCE_MS 10
#define TELEMETRY_MS 100
#define SERVO_MIN_US 400
#define SERVO_MAX_US 2600
#define GRIPPER 5
#define GRIPPER_OPEN_US 2000    // placeholder: measure on your gripper
// Rest pose in microseconds, same order as the servo channels. Placeholders:
// measure these on the real arm before using the park move.
#define REST_POSE {1352, 1500, 1500, 1500, 1500, 2000}
