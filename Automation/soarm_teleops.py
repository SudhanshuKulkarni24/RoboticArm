#!/usr/bin/env python3
"""
SO-ARM local teleoperation script (keyboard or DualSense).
- Direct SCServo control (Feetech STS series) via scservo_sdk
- No phosphobot server required
- Joint-space incremental control with soft limits

Keys:
  Mode select at start: [1]=Keyboard, [2]=DualSense
Keyboard controls:
  1..6 : select joint (toggles active joint)
  ArrowUp/W  : + increment on active joint
  ArrowDown/S: - increment on active joint
  A/D or Left/Right arrows can be bound to other joints if desired
  [ and ] : decrease/increase step size
  Space : stop all motion (hold current positions)
  R : go to 'home' pose
  Q : quit

DualSense (via pygame):
  Left stick X -> Joint 1 (base)
  Left stick Y -> Joint 2 (shoulder)
  Right stick Y -> Joint 3 (elbow)
  Right stick X -> Joint 4 (wrist pitch)
  L2 trigger    -> Joint 5 (-)
  R2 trigger    -> Joint 5 (+)
  Cross (X)     -> Gripper close  (Joint 6 +)
  Circle (O)    -> Gripper open   (Joint 6 -)
  Triangle      -> Home pose
  Square        -> Stop (hold)
  Options       -> Quit

Adjust JOINT_IDS, PORT, BAUD, limits, and home_pose for your arm.
"pip install feetech-servo-sdk pygame"
"""

import sys
import time
import math
import threading

# -------- Keyboard helpers (cross-platform) ----------
try:
    import msvcrt  # Windows
    _ON_WINDOWS = True
except ImportError:
    import tty, termios, sys, select  # POSIX
    _ON_WINDOWS = False

# -------- Controller via pygame ----------
try:
    import pygame
    _HAVE_PYGAME = True
except Exception:
    _HAVE_PYGAME = False

# -------- Feetech SCServo SDK ----------
# The module might be named scservo_sdk or scservo-sdk depending on packaging
try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    # Fallback alt name
    from scservo_sdk import PortHandler, PacketHandler

# =============== USER CONFIG =================
PORT = "/dev/tty.usbmodem59700731201"          # e.g., "COM3" on Windows, "/dev/ttyUSB0" or "/dev/ttyAMA0" on Linux
BAUD = 1000000         # Common for STS3215; try 115200 if you get timeouts
PROTOCOL_VERSION = 0   # Feetech SCServo uses its own single-byte protocol; keep 0 (SDK abstracts it)

# Your SO-ARM joint IDs (update if needed)
JOINT_IDS = [1, 2, 3, 4, 5, 6]   # base, shoulder, elbow, wrist_pitch, wrist_roll (or yaw), gripper

# Position units: Feetech STS3215 are 0..4095 for 0..360deg (12-bit), center ~ 2048
# Define safe software limits per joint (min, max)
SOFT_LIMITS = {
    1: (0, 4000),
    2: (400, 3600),
    3: (400, 3600),
    4: (400, 3600),
    5: (400, 3600),
    6: (1500, 3000),   # gripper travel – tweak for your linkage
}

# Default home pose (conservative)
HOME_POSE = {
    1: 2048,  # base center
    2: 2048,  # shoulder mid
    3: 2048,  # elbow mid
    4: 2048,  # wrist pitch mid
    5: 2048,  # wrist roll mid
    6: 2200,  # gripper slightly open
}

# Motion increments & update timing
KEYBOARD_STEP = 18        # ~1.6 deg if 4096 ticks ~ 360 deg; adjust at runtime with [ ]
GAMEPAD_GAIN = 120        # ticks per second at full stick deflection
TRIGGER_GAIN = 140        # ticks per second for triggers -> joint 5
GRIPPER_STEP = 20
CTRL_RATE_HZ = 60

# Feetech addresses (from SDK docs). For STS series:
ADDR_GOAL_POSITION = 0x2A   # Goal Position (2 bytes) - check your servo model
ADDR_PRESENT_POSITION = 0x38  # Present Position (2 bytes)
ADDR_TORQUE_ENABLE = 0x18
ADDR_SPEED = 0x20            # Moving speed (2 bytes)
# ============================================


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class Arm:
    def __init__(self, port:str, baud:int):
        self.port_handler = PortHandler(port)
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        if not self.port_handler.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baud {baud}")

        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Cache current goal (start from home)
        self.goal = dict(HOME_POSE)

        # Enable torque
        for jid in JOINT_IDS:
            self.write8(jid, ADDR_TORQUE_ENABLE, 1)
            # Optionally set a moderate speed so small steps move smoothly
            self.write16(jid, ADDR_SPEED, 200)

        # Move to home slowly on connect
        self.goto_pose(HOME_POSE, duration=1.0)

    # ------- Low-level helpers (8/16-bit writes & reads) ----------
    def write8(self, servo_id:int, addr:int, value:int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, addr, value)
        if dxl_comm_result != 0:
            print(f"[WARN] write8 id{servo_id} addr{addr} err={dxl_comm_result}/{dxl_error}")

    def write16(self, servo_id:int, addr:int, value:int):
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, servo_id, addr, value)
        if dxl_comm_result != 0:
            print(f"[WARN] write16 id{servo_id} addr{addr} val={value} err={dxl_comm_result}/{dxl_error}")

    def read16(self, servo_id:int, addr:int, default=None):
        value, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, servo_id, addr)
        if dxl_comm_result != 0:
            return default
        return value

    # ------- Position control ----------
    def set_goal(self, servo_id:int, ticks:int):
        lo, hi = SOFT_LIMITS.get(servo_id, (0, 4095))
        ticks = clamp(int(ticks), lo, hi)
        self.goal[servo_id] = ticks
        self.write16(servo_id, ADDR_GOAL_POSITION, ticks)

    def incr(self, servo_id:int, delta:int):
        cur = self.goal.get(servo_id, HOME_POSE.get(servo_id, 2048))
        self.set_goal(servo_id, cur + delta)

    def goto_pose(self, pose:dict, duration:float=0.8):
        """Linearly interpolate from current goals to target pose."""
        steps = max(1, int(duration * CTRL_RATE_HZ))
        start = {j: self.goal.get(j, HOME_POSE[j]) for j in JOINT_IDS}
        for i in range(1, steps+1):
            a = i/steps
            for j in JOINT_IDS:
                tgt = int(round((1-a) * start[j] + a * pose[j]))
                self.set_goal(j, tgt)
            time.sleep(1.0/CTRL_RATE_HZ)

    def relax(self):
        for jid in JOINT_IDS:
            self.write8(jid, ADDR_TORQUE_ENABLE, 0)

    def close(self):
        try:
            self.relax()
        finally:
            try:
                self.port_handler.closePort()
            except Exception:
                pass


# --------- Keyboard mode ----------
class Keyboard:
    def __init__(self):
        self.selected_joint = JOINT_IDS[0]
        self.step = KEYBOARD_STEP

    def get_key(self, timeout=0.016):
        if _ON_WINDOWS:
            start = time.time()
            while time.time() - start < timeout:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    return ch
                time.sleep(0.001)
            return None
        else:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                r, _, _ = select.select([sys.stdin], [], [], timeout)
                if r:
                    return sys.stdin.read(1)
                return None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def loop(self, arm:Arm):
        print("[Keyboard] Active joint starts at 1. Keys: 1..6 select joint, arrows/W/S to move, [ ] step, R home, Space stop, Q quit")
        running = True
        while running:
            ch = self.get_key()
            if not ch:
                continue

            if ch.lower() in "123456":
                self.selected_joint = int(ch)
                print(f"Selected joint {self.selected_joint}")
            elif ch in ('\x1b',):  # escape on some terms
                running = False
            elif ch.lower() == 'q':
                running = False
            elif ch.lower() == 'r':
                print("Going home")
                arm.goto_pose(HOME_POSE, duration=1.0)
            elif ch == ' ':  # stop -> hold by writing current goals again
                print("Hold position")
                for j in JOINT_IDS:
                    arm.set_goal(j, arm.goal[j])
            elif ch == '[':
                self.step = max(1, self.step - 1)
                print(f"Step: {self.step}")
            elif ch == ']':
                self.step += 1
                print(f"Step: {self.step}")
            else:
                # Arrow keys handling (Windows returns '\xe0' then code; POSIX often ESC sequences)
                # Provide simple WASD as well
                if ch.lower() in ('w',):  # + on selected
                    arm.incr(self.selected_joint, +self.step)
                elif ch.lower() in ('s',):  # -
                    arm.incr(self.selected_joint, -self.step)
                else:
                    # Attempt to parse arrow codes
                    if _ON_WINDOWS and ch == '\xe0':
                        nxt = msvcrt.getwch()
                        if nxt == 'H':   # up
                            arm.incr(self.selected_joint, +self.step)
                        elif nxt == 'P': # down
                            arm.incr(self.selected_joint, -self.step)
                        elif nxt == 'K': # left (optional: map to previous joint)
                            pass
                        elif nxt == 'M': # right
                            pass
                    # On POSIX, many terminals send '\x1b[A' etc. We already consumed '\x1b', ignore for brevity
        print("Exiting keyboard mode...")


# --------- DualSense (pygame) mode ----------
class DualSense:
    def __init__(self, deadzone=0.15):
        if not _HAVE_PYGAME:
            raise RuntimeError("pygame not installed. Run: pip install pygame")
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No gamepad detected. Connect DualSense (USB recommended) and try again.")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.deadzone = deadzone
        self.last_time = time.time()

        print(f"[Gamepad] Using: {self.js.get_name()}")

    def _dz(self, v:float):
        return 0.0 if abs(v) < self.deadzone else v

    def loop(self, arm:Arm):
        clock = pygame.time.Clock()
        print("[Gamepad] Sticks: J1/J2/J3/J4, R2->gripper open, L2->gripper close, X/O->unused, Triangle->home, Square->hold, Options->quit")
        running = True
        while running:
            dt = clock.tick(CTRL_RATE_HZ) / 200.0
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    # PS mapping varies by OS; these are common indices:
                    # 0=Square, 1=Cross(X), 2=Circle(O), 3=Triangle, 9=Options
                    if event.button == 9:   # Options -> quit
                        running = False
                    elif event.button == 3: # Triangle -> home
                        arm.goto_pose(HOME_POSE, duration=1.0)
                    elif event.button == 0: # Square -> hold
                        for j in JOINT_IDS:
                            arm.set_goal(j, arm.goal[j])
                    # X and O buttons are now unused since gripper is controlled by triggers
                    # elif event.button == 1: # Cross (X) -> unused
                    # elif event.button == 2: # Circle (O) -> unused

            # Axes:
            # Common DualSense mapping:
            # 0: LX (-left,+right), 1: LY (-up,+down)
            # 2: RX, 3: RY
            # 4: L2 (0..1 or -1..1 depending OS)
            # 5: R2 (0..1 or -1..1)
            lx = self._dz(self.js.get_axis(0))
            ly = self._dz(self.js.get_axis(1))
            rx = self._dz(self.js.get_axis(2))
            ry = self._dz(self.js.get_axis(3))

            # Triggers can be -1..1 (Linux) or 0..1 (Windows/Mac). Normalize to 0..1.
            l2 = self.js.get_axis(4)
            r2 = self.js.get_axis(5)
            def norm_trigger(v):
                # if in [-1,1], map to [0,1]; if already [0,1], keep
                return (v + 1) / 2.0 if -1.0 <= v <= 1.0 else max(0.0, min(1.0, v))
            l2n = norm_trigger(l2)
            r2n = norm_trigger(r2)

            # Convert axes to increments per frame
            arm.incr(1, int(lx * GAMEPAD_GAIN * dt))   # base
            arm.incr(2, int(-ly * GAMEPAD_GAIN * dt))  # shoulder (invert so up=+)
            arm.incr(3, int(-ry * GAMEPAD_GAIN * dt))  # elbow (right stick Y)
            arm.incr(4, int(rx * GAMEPAD_GAIN * dt))   # wrist pitch
            
            # Gripper control: R2 opens (+), L2 closes (-)
            gripper_delta = (r2n - l2n) * GRIPPER_STEP * dt * 20
            if abs(gripper_delta) > 0.01:  # deadzone to avoid tiny movements
                arm.incr(6, int(gripper_delta))
            
            # Wrist roll is no longer controlled by triggers - you may want to map it elsewhere
            # arm.incr(5, int((r2n - l2n) * TRIGGER_GAIN * dt))  # wrist roll (commented out)

        print("Exiting gamepad mode...")
        pygame.quit()


def select_mode():
    print("\nSelect mode:\n  [1] Keyboard\n  [2] DualSense (pygame)\n")
    choice = input("Enter 1 or 2: ").strip()
    return choice


def main():
    print("=== SO-ARM Teleop (SCServo) ===")
    print(f"Opening {PORT} @ {BAUD}")
    arm = Arm(PORT, BAUD)
    try:
        mode = select_mode()
        if mode == '1':
            Keyboard().loop(arm)
        elif mode == '2':
            DualSense().loop(arm)
        else:
            print("Unknown selection. Exiting.")
    finally:
        arm.close()
        print("Goodbye.")

if __name__ == "__main__":
    main()
