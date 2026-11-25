#!/usr/bin/env python3
"""Board-in-the-loop check: simulated Uno <-> real bridge <-> real supervisor.

    cd ~/robot_arm_ws/firmware/host && make && python3 bridge_check.py

Runs on its own ROS domain, with no Gazebo. uno_sim is the real firmware core
(core.cpp) behind a virtual serial port; this script plays the operator:
presses the E-stop, blocks beams, pushes the current up, mutes the Uno, and
kills the supervisor, then checks what the cell does.
"""

import os
import re
import signal
import subprocess
import sys
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '78'
import rclpy                                    # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, Float64, String  # noqa: E402
from std_srvs.srv import Trigger                # noqa: E402

from factory_interfaces.msg import CellStatus   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
procs, results = [], []
uno = {'state': None, 'fault': None, 'relay': None, 'srv0': None, 'log': []}


def check(name, ok, detail=''):
    results.append(ok)
    print(f'{"PASS" if ok else "FAIL"}  {name}' + (f'  ({detail})' if detail and not ok else ''), flush=True)


def start(cmd, **kw):
    p = subprocess.Popen(cmd, start_new_session=True, **kw)
    procs.append(p)
    return p


def stop_all():
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


def reader(p):
    for line in p.stdout:
        m = re.match(r'state (\d) fault (\d) relay (\d) srv0 (\d+)', line)
        if m:
            uno.update(state=int(m[1]), fault=int(m[2]), relay=int(m[3]), srv0=int(m[4]))
            uno['log'].append((time.monotonic(), dict(uno)))


def wait(pred, timeout, step=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def main():
    subprocess.run(['make', 'uno_sim'], cwd=HERE, check=True, capture_output=True)
    sim = start([os.path.join(HERE, 'uno_sim')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    port = sim.stdout.readline().strip()
    threading.Thread(target=reader, args=(sim,), daemon=True).start()
    poke = lambda s: (sim.stdin.write(s + '\n'), sim.stdin.flush())   # noqa: E731

    rclpy.init()
    node = rclpy.create_node('bridge_check')
    st = {'status': None, 'ir1_hits': 0, 'speed': 0.0, 'estop': None, 'fault': None, 'wd': None, 'relay': None}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CellStatus, '/cell/status', lambda m: st.__setitem__('status', m), 10)
    node.create_subscription(Bool, '/ir1', lambda m: st.__setitem__('ir1_hits', st['ir1_hits'] + m.data), latched)
    node.create_subscription(Float64, '/conveyor/speed', lambda m: st.__setitem__('speed', m.data), 10)
    node.create_subscription(Bool, '/estop', lambda m: st.__setitem__('estop', m.data), latched)
    node.create_subscription(Bool, '/io/watchdog_ok', lambda m: st.__setitem__('wd', m.data), latched)
    node.create_subscription(Bool, '/relay/state', lambda m: st.__setitem__('relay', m.data), latched)
    node.create_subscription(String, '/io/fault', lambda m: st.__setitem__('fault', m.data), latched)
    reset = node.create_client(Trigger, '/cell/reset')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    state = lambda: st['status'].state if st['status'] else None   # noqa: E731
    fault = lambda: st['status'].fault if st['status'] else ''     # noqa: E731

    def do_reset():
        reset.wait_for_service(timeout_sec=5)
        f = reset.call_async(Trigger.Request())
        wait(f.done, 5)
        return f.result().message if f.done() else 'no answer'

    start(['ros2', 'run', 'factory_control', 'uno_bridge_node', '--ros-args',
           '-p', f'port:={port}', '-p', 'send_home:=true'])
    sup = start(['ros2', 'run', 'factory_control', 'supervisor_node'])

    # 1. start-up: heartbeat first, home pose, then the relay and the belt
    check('start-up: the Uno reaches RUN with the relay closed and the supervisor RUNNING',
          wait(lambda: uno['state'] == 1 and uno['relay'] == 1 and state() == 'RUNNING', 25),
          f'uno={uno["state"]} relay={uno["relay"]} sup={state()}')
    check('the belt ramps up to 30 mm/s', wait(lambda: abs(st['speed'] - 30.0) < 0.5, 5), f'{st["speed"]}')

    # 2. beams
    n0 = st['ir1_hits']
    poke('ir1 1'); time.sleep(0.15); poke('ir1 0')
    check('a beam IR1 pass reaches the supervisor as one entry',
          wait(lambda: st['status'] and st['status'].entered == 1, 3), f'entered={st["status"].entered if st["status"] else None}, hits {st["ir1_hits"] - n0}')
    poke('ir2 1')
    check('IR2 stops the belt for a pick', wait(lambda: state() in ('STOPPING', 'PICKING') and st['speed'] < 1.0, 6), f'{state()} {st["speed"]}')
    time.sleep(0.5); poke('ir2 0')
    check('with no detection the supervisor gives up the pick and restarts the belt',
          wait(lambda: state() == 'RUNNING' and abs(st['speed'] - 30.0) < 0.5, 12), f'{state()} {st["speed"]}')

    # 3. E-stop
    poke('estop 1')
    check('E-stop: the relay opens at once and the belt stops',
          wait(lambda: uno['relay'] == 0 and st['speed'] == 0.0, 1.0), f'relay={uno["relay"]} speed={st["speed"]}')
    check('E-stop: the supervisor goes to ESTOP', wait(lambda: state() == 'ESTOP', 2), state())
    poke('estop 0'); time.sleep(1.0)
    check('releasing the E-stop alone does not restart anything', uno['relay'] == 0 and state() == 'ESTOP', f'relay={uno["relay"]} {state()}')
    msg = do_reset()
    check('reset clears the supervisor and the Uno and the line runs again',
          wait(lambda: state() == 'RUNNING' and uno['state'] == 1 and uno['relay'] == 1, 8), f'{msg}: {state()} uno={uno["state"]}')

    # 4. servo overcurrent
    poke('ma 7000')
    check('servo overcurrent: the Uno faults and the supervisor stops the cell',
          wait(lambda: uno['fault'] == 3 and state() == 'FAULT' and st['speed'] == 0.0, 3), f'uno fault={uno["fault"]} sup={state()} {fault()}')
    poke('ma 500'); time.sleep(0.5)
    check('the fault is not cleared by itself', state() == 'FAULT', state())
    msg = do_reset()
    check('reset after the current is back to normal', wait(lambda: state() == 'RUNNING' and uno['state'] == 1, 8), f'{msg}: {state()} uno={uno["state"]}')

    # 5. serial link lost: the Uno stops answering AND stops listening
    poke('mute 1')
    check('serial link lost: the bridge reports it and the supervisor faults',
          wait(lambda: st['wd'] is False and state() == 'FAULT', 4), f'wd={st["wd"]} {state()} {fault()}')
    check('the Uno parks and cuts motor power on its own (no heartbeat for 2 s, then 3 s park)',
          wait(lambda: uno['state'] == 3 and uno['relay'] == 0 and uno['srv0'] == 1352, 9), f'state={uno["state"]} relay={uno["relay"]} srv0={uno["srv0"]}')
    poke('mute 0')
    msg = do_reset()
    check('after the link is back, reset restores the line', wait(lambda: state() == 'RUNNING' and uno['state'] == 1, 12), f'{msg}: {state()} uno={uno["state"]}')

    # 6. the supervisor (the computer side) dies
    t0 = time.monotonic()
    os.killpg(sup.pid, signal.SIGKILL)
    check('supervisor dies: the Uno parks the arm after its 2 s heartbeat window', wait(lambda: uno['state'] == 2, 4), f'{uno["state"]}')
    check('then opens the relay once parked', wait(lambda: uno['relay'] == 0 and uno['state'] == 3, 6), f'state={uno["state"]} relay={uno["relay"]} after {time.monotonic() - t0:.1f}s')

    ok = all(results)
    print(f'\n{sum(results)}/{len(results)} checks passed')
    stop_all()
    return 0 if ok else 1


if __name__ == '__main__':
    try:
        code = main()
    finally:
        stop_all()
    sys.exit(code)
