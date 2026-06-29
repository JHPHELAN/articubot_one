#!/usr/bin/env python3
"""Headlights and safety-beacon GPIO node for Stormy the Stingray.

Hardware (May 2026 PCB build):
    - Custom indicators PCB with two IRLZ44N N-channel MOSFETs driven
      directly by Pi GPIO (no NPN inverter stage).
    - GPIO HIGH = MOSFET ON = load powered. So both channels are
      logically active-high (``headlights_active_low=false``,
      ``beacon_active_low=false``).
    - With the Pi off, or before this node starts driving the pins, the
      MOSFET gates float/are pulled low, so all loads are OFF by default.
      The optional ``headlights_startup_on_sec`` window (default 3.0 s)
      gives a brief boot-time "running light" by turning the headlights
      on at node start and back off after the delay.

Headlights:
    - Driven by IRLZ44N #1 on the indicators PCB. GPIO HIGH = bulbs ON.
    - Controlled by joystick buttons. Default: A (button 0) = ON,
      B (button 1) = OFF. Idempotent (re-pressing has no effect).

Safety beacon (LED + audio warning):
    - LED: driven by IRLZ44N #2 on the indicators PCB. GPIO HIGH = LED on.
      (Replaces the earlier 2N3904 small-signal driver.)
    - Audio: a WAV is played via ``aplay`` through the USB audio dongle
      every ``beacon_sound_repeat_sec`` seconds while the beacon is active
      AND the sound enable toggle is on. The LED phase is re-anchored to
      each ``aplay`` start so blink and audio stay synchronized.
      Replaces the old GPIO 22 piezo buzzer; GPIO 22 is now the LED line.
    - Auto-on when ``/cmd_vel`` shows commanded motion above a small
      threshold. Stays on for ``beacon_linger_sec`` after motion stops.
    - Manual light toggle via joystick button (default Y = button 3).
    - Manual sound enable toggle via separate joystick button
      (default X = button 2, blue). When sound is disabled the lights
      still flash silently. Default is sound DISABLED at startup; set
      ``beacon_sound_default_on`` to true to restore the old behavior.

Nav2 trouble alert:
    - Plays ``alert_sound_path`` (default danger.wav) when Nav2 reports
      trouble. Two trigger sources, each enable-able independently:
        * Goal aborted: any NavigateToPose goal returning STATUS_ABORTED.
        * Recovery firing: a Behavior Tree node from
          ``alert_recovery_node_names`` enters RUNNING (e.g. ClearCostmap,
          Wait, Spin).
    - Throttled by ``alert_min_interval_sec`` so a flood of recovery
      events does not flood the speaker.
    - Shares the single aplay subprocess slot with the beacon and
      preempts any in-progress beacon clip; beacon clips will not start
      while an alert clip is playing.
"""

from __future__ import annotations

import os
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

try:
    from action_msgs.msg import GoalStatusArray
    ACTION_MSGS_AVAILABLE = True
except Exception as exc:  # pragma: no cover
    ACTION_MSGS_AVAILABLE = False
    _ACTION_MSGS_IMPORT_ERROR = exc

try:
    from nav2_msgs.msg import BehaviorTreeLog
    NAV2_MSGS_AVAILABLE = True
except Exception as exc:  # pragma: no cover
    NAV2_MSGS_AVAILABLE = False
    _NAV2_MSGS_IMPORT_ERROR = exc

try:
    from gpiozero import LED, Device
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()
    GPIO_AVAILABLE = True
except Exception as exc:  # pragma: no cover
    GPIO_AVAILABLE = False
    _GPIO_IMPORT_ERROR = exc


class IndicatorsNode(Node):
    def __init__(self) -> None:
        super().__init__('indicators_node')

        self.declare_parameter('headlights_pin', 17)
        self.declare_parameter('beacon_pin', 27)
        self.declare_parameter('headlights_active_low', True)
        self.declare_parameter('beacon_active_low', False)
        self.declare_parameter('beacon_led_on_sec', 0.5)
        self.declare_parameter('beacon_led_off_sec', 0.5)

        # Audio warning (replaces GPIO 22 piezo buzzer).
        self.declare_parameter(
            'beacon_sound_path',
            '/home/ubuntu/wav/r2d2-chatter-loud.wav')
        self.declare_parameter('beacon_sound_repeat_sec', 3.0)
        self.declare_parameter('beacon_sound_device', '')  # empty = aplay default

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('headlights_on_button', 0)   # A (green)
        self.declare_parameter('headlights_off_button', 1)  # B (red)
        self.declare_parameter('beacon_manual_button', 3)   # Y (yellow) - lights only
        self.declare_parameter('beacon_sound_button', 2)    # X (blue)   - sound only
        self.declare_parameter('beacon_sound_default_on', False)

        self.declare_parameter('linear_threshold', 0.01)
        self.declare_parameter('angular_threshold', 0.05)
        self.declare_parameter('beacon_linger_sec', 2.0)

        self.declare_parameter('headlights_startup_on_sec', 3.0)
        self.declare_parameter('mock_gpio', False)

        # Nav2 trouble alert
        self.declare_parameter(
            'alert_sound_path',
            '/home/ubuntu/wav/danger.wav')
        self.declare_parameter('alert_sound_device', '')  # empty = follow beacon device
        self.declare_parameter('alert_min_interval_sec', 8.0)
        self.declare_parameter('alert_on_goal_aborted', True)
        self.declare_parameter('alert_on_recovery', True)
        self.declare_parameter('bt_log_topic', '/behavior_tree_log')
        self.declare_parameter(
            'nav_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter(
            'alert_recovery_node_names',
            ['ClearEntireCostmap', 'ClearLocalCostmap', 'ClearGlobalCostmap',
             'Wait', 'Spin', 'BackUp'])

        gp = self.get_parameter
        self._hl_pin = int(gp('headlights_pin').value)
        self._bn_pin = int(gp('beacon_pin').value)
        self._hl_active_low = bool(gp('headlights_active_low').value)
        self._bn_active_low = bool(gp('beacon_active_low').value)
        self._led_on_sec = float(gp('beacon_led_on_sec').value)
        self._led_off_sec = float(gp('beacon_led_off_sec').value)

        self._sound_path = str(gp('beacon_sound_path').value)
        self._sound_repeat = float(gp('beacon_sound_repeat_sec').value)
        self._sound_device = str(gp('beacon_sound_device').value)

        self._joy_topic = str(gp('joy_topic').value)
        self._cmd_vel_topic = str(gp('cmd_vel_topic').value)

        self._btn_on = int(gp('headlights_on_button').value)
        self._btn_off = int(gp('headlights_off_button').value)
        self._btn_beacon = int(gp('beacon_manual_button').value)
        self._btn_sound = int(gp('beacon_sound_button').value)
        self._sound_enabled = bool(gp('beacon_sound_default_on').value)

        self._lin_thresh = float(gp('linear_threshold').value)
        self._ang_thresh = float(gp('angular_threshold').value)
        self._linger = float(gp('beacon_linger_sec').value)

        self._startup_on = float(gp('headlights_startup_on_sec').value)
        self._mock = bool(gp('mock_gpio').value) or not GPIO_AVAILABLE

        self._alert_sound_path = str(gp('alert_sound_path').value)
        _alert_dev = str(gp('alert_sound_device').value)
        self._alert_sound_device = _alert_dev if _alert_dev else self._sound_device
        self._alert_min_interval = float(gp('alert_min_interval_sec').value)
        self._alert_on_aborted = bool(gp('alert_on_goal_aborted').value)
        self._alert_on_recovery = bool(gp('alert_on_recovery').value)
        self._bt_log_topic = str(gp('bt_log_topic').value)
        self._nav_status_topic = str(gp('nav_status_topic').value)
        self._recovery_node_names = list(gp('alert_recovery_node_names').value)

        # State
        self._headlights_on = self._startup_on > 0.0  # ON during startup window
        self._beacon_on = False           # logical beacon-active (LED+sound should pattern)
        self._led_phase_on = False        # current physical LED output
        self._led_phase_time = None       # rclpy Time of last LED phase change
        self._beacon_manual = False
        self._last_motion_time = None
        self._prev_buttons: list[int] = []
        self._start_time = None  # set after first tick

        # Audio playback state
        self._sound_proc: subprocess.Popen | None = None
        self._sound_kind: str | None = None  # 'beacon' or 'alert' when proc alive
        self._sound_last_start_time = None  # rclpy Time of last aplay launch (beacon)
        self._alert_pending = False
        self._alert_last_time = None  # rclpy Time of last alert play
        self._reported_aborts: set = set()  # dedupe goal IDs
        self._aplay_path = shutil.which('aplay')
        if self._aplay_path is None:
            self.get_logger().warn(
                'aplay not found on PATH; beacon sound disabled.')
        elif not os.path.isfile(self._sound_path):
            self.get_logger().warn(
                f'beacon_sound_path not found: {self._sound_path}; '
                'beacon sound disabled.')
        if self._aplay_path is not None and not os.path.isfile(self._alert_sound_path):
            self.get_logger().warn(
                f'alert_sound_path not found: {self._alert_sound_path}; '
                'Nav2 alert sound disabled.')

        if self._mock:
            self.get_logger().warn(
                'GPIO not available or mock_gpio=true; running in simulation mode.'
            )
            if not GPIO_AVAILABLE:
                self.get_logger().warn(f'gpiozero import error: {_GPIO_IMPORT_ERROR}')
            self._hl = None
            self._bn = None
        else:
            self._hl = LED(self._hl_pin,
                           active_high=not self._hl_active_low,
                           initial_value=self._headlights_on)
            self._bn = LED(self._bn_pin,
                           active_high=not self._bn_active_low,
                           initial_value=False)

        self._apply_headlights()
        self._apply_beacon_outputs()

        self.create_subscription(Joy, self._joy_topic, self._joy_cb, 10)
        self.create_subscription(TwistStamped, self._cmd_vel_topic, self._cmd_vel_cb, 10)

        # Nav2 trouble subscriptions (best-effort; missing message packages just disable the feature).
        if self._alert_on_aborted:
            if ACTION_MSGS_AVAILABLE:
                self.create_subscription(
                    GoalStatusArray, self._nav_status_topic,
                    self._nav_status_cb, 10)
            else:
                self.get_logger().warn(
                    f'action_msgs not importable ({_ACTION_MSGS_IMPORT_ERROR}); '
                    'goal-aborted alert disabled.')
        if self._alert_on_recovery:
            if NAV2_MSGS_AVAILABLE:
                self.create_subscription(
                    BehaviorTreeLog, self._bt_log_topic,
                    self._bt_log_cb, 10)
            else:
                self.get_logger().warn(
                    f'nav2_msgs not importable ({_NAV2_MSGS_IMPORT_ERROR}); '
                    'recovery alert disabled.')

        self._pub_headlights = self.create_publisher(Bool, '/headlights/state', 10)
        self._pub_beacon = self.create_publisher(Bool, '/beacon/state', 10)

        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'indicators_node ready: headlights pin {self._hl_pin} '
            f'(active_low={self._hl_active_low}), beacon pin {self._bn_pin} '
            f'(active_low={self._bn_active_low}), startup_on={self._startup_on}s, '
            f'mock={self._mock}'
        )

    def _joy_cb(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        prev = self._prev_buttons
        self._prev_buttons = buttons

        def pressed(idx: int) -> bool:
            if idx < 0 or idx >= len(buttons):
                return False
            was = prev[idx] if idx < len(prev) else 0
            return buttons[idx] == 1 and was == 0

        if pressed(self._btn_on) and not self._headlights_on:
            self._headlights_on = True
            self._apply_headlights()
            self.get_logger().info('Headlights ON')
        elif pressed(self._btn_off) and self._headlights_on:
            self._headlights_on = False
            self._apply_headlights()
            self.get_logger().info('Headlights OFF')

        if pressed(self._btn_beacon):
            self._beacon_manual = not self._beacon_manual
            self.get_logger().info(
                f'Beacon manual override {"ON" if self._beacon_manual else "OFF"}'
            )

        if pressed(self._btn_sound):
            self._sound_enabled = not self._sound_enabled
            self.get_logger().info(
                f'Beacon sound {"ENABLED" if self._sound_enabled else "DISABLED"}'
            )
            if not self._sound_enabled:
                # Stop any in-progress beacon clip immediately so the operator
                # gets quiet on the next button press. Alert clips are left
                # alone so Nav2 trouble notifications still get through.
                if (self._sound_proc is not None
                        and self._sound_kind == 'beacon'
                        and self._sound_proc.poll() is None):
                    try:
                        self._sound_proc.terminate()
                        try:
                            self._sound_proc.wait(timeout=0.2)
                        except Exception:
                            self._sound_proc.kill()
                    except Exception:
                        pass
                    self._sound_proc = None
                    self._sound_kind = None
            else:
                # Force a fresh clip on the next tick instead of waiting out
                # the throttle window from the previous enable cycle.
                self._sound_last_start_time = None

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        t = msg.twist
        moving = (abs(t.linear.x) > self._lin_thresh or
                  abs(t.linear.y) > self._lin_thresh or
                  abs(t.angular.z) > self._ang_thresh)
        if moving:
            self._last_motion_time = self.get_clock().now()

    def _nav_status_cb(self, msg) -> None:
        # GoalStatusArray: status_list of GoalStatus, status==6 is ABORTED.
        STATUS_ABORTED = 6
        for s in msg.status_list:
            if s.status != STATUS_ABORTED:
                continue
            try:
                gid = bytes(s.goal_info.goal_id.uuid)
            except Exception:
                gid = None
            if gid is not None and gid in self._reported_aborts:
                continue
            if gid is not None:
                self._reported_aborts.add(gid)
                # Cap memory growth across long sessions.
                if len(self._reported_aborts) > 256:
                    self._reported_aborts = set(list(self._reported_aborts)[-128:])
            self._alert_pending = True
            self.get_logger().info('Nav2 goal ABORTED -> danger alert queued.')
            break

    def _bt_log_cb(self, msg) -> None:
        # Trigger when a recovery node transitions into RUNNING.
        for ev in msg.event_log:
            if ev.current_status != 'RUNNING' or ev.previous_status == 'RUNNING':
                continue
            name = ev.node_name
            for needle in self._recovery_node_names:
                if needle and needle in name:
                    self._alert_pending = True
                    self.get_logger().info(
                        f'Nav2 recovery firing ({name}) -> danger alert queued.')
                    return

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now

        # Auto-off headlights after startup window (one-shot).
        if self._startup_on > 0.0 and self._headlights_on:
            age = (now - self._start_time).nanoseconds * 1e-9
            if age >= self._startup_on:
                self._headlights_on = False
                self._apply_headlights()
                self._startup_on = 0.0  # disable further auto-off
                self.get_logger().info('Boot running-light period ended; headlights OFF.')

        beacon_should_be_on = self._beacon_manual
        if not beacon_should_be_on and self._last_motion_time is not None:
            age = (now - self._last_motion_time).nanoseconds * 1e-9
            if age <= self._linger:
                beacon_should_be_on = True

        if beacon_should_be_on != self._beacon_on:
            self._beacon_on = beacon_should_be_on
            if self._beacon_on:
                # Start LED in ON phase, aligned to now.
                self._led_phase_on = True
                self._led_phase_time = now
                # Force sound to play immediately on activation.
                self._sound_last_start_time = None
            else:
                self._led_phase_on = False
            self._apply_beacon_outputs()

        # LED blink pattern while beacon is active.
        if self._beacon_on:
            dur = self._led_on_sec if self._led_phase_on else self._led_off_sec
            if dur > 0.0 and self._led_phase_time is not None:
                age = (now - self._led_phase_time).nanoseconds * 1e-9
                if age >= dur:
                    self._led_phase_on = not self._led_phase_on
                    self._led_phase_time = now
                    self._apply_beacon_outputs()

            # Periodic WAV playback via aplay.
            self._maybe_play_sound(now)
        else:
            # Reap any finished aplay process when beacon is idle.
            if self._sound_proc is not None and self._sound_proc.poll() is not None:
                self._sound_proc = None
                self._sound_kind = None

        # Nav2 trouble alert (preempts beacon clip).
        if self._alert_pending:
            if self._maybe_play_alert(now):
                self._alert_pending = False

        m = Bool(); m.data = self._headlights_on
        self._pub_headlights.publish(m)
        m2 = Bool(); m2.data = self._beacon_on
        self._pub_beacon.publish(m2)

    def _apply_headlights(self) -> None:
        if self._hl is None:
            return
        if self._headlights_on:
            self._hl.on()
        else:
            self._hl.off()

    def _apply_beacon_outputs(self) -> None:
        if self._bn is not None:
            if self._beacon_on and self._led_phase_on:
                self._bn.on()
            else:
                self._bn.off()

    def _maybe_play_sound(self, now) -> None:
        if not self._sound_enabled:
            return
        if self._aplay_path is None or not os.path.isfile(self._sound_path):
            return
        # Don't overlap: skip if previous clip is still playing.
        if self._sound_proc is not None:
            if self._sound_proc.poll() is None:
                return
            self._sound_proc = None
            self._sound_kind = None
        # Throttle by repeat interval.
        if self._sound_last_start_time is not None:
            age = (now - self._sound_last_start_time).nanoseconds * 1e-9
            if age < self._sound_repeat:
                return
        cmd = [self._aplay_path, '-q']
        if self._sound_device:
            cmd += ['-D', self._sound_device]
        cmd.append(self._sound_path)
        try:
            self._sound_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL)
            self._sound_kind = 'beacon'
            self._sound_last_start_time = now
            # Re-anchor LED phase to the audio start so blink and tones stay
            # synchronised across many loops (aplay startup latency would
            # otherwise let the LED gradually lead the sound).
            self._led_phase_on = True
            self._led_phase_time = now
            self._apply_beacon_outputs()
        except Exception as exc:
            self.get_logger().warn(f'aplay launch failed: {exc}')

    def _maybe_play_alert(self, now) -> bool:
        """Play danger.wav if cooldown allows. Returns True if handled."""
        if self._aplay_path is None or not os.path.isfile(self._alert_sound_path):
            return True  # consume the trigger; alert is configured-disabled
        # Cooldown between alerts.
        if self._alert_last_time is not None:
            age = (now - self._alert_last_time).nanoseconds * 1e-9
            if age < self._alert_min_interval:
                return True  # consume; we just won't play this one
        # Preempt any currently-playing beacon clip; never preempt an alert.
        if self._sound_proc is not None:
            if self._sound_proc.poll() is None:
                if self._sound_kind == 'alert':
                    return False  # try again next tick
                try:
                    self._sound_proc.terminate()
                    self._sound_proc.wait(timeout=0.2)
                except Exception:
                    try:
                        self._sound_proc.kill()
                    except Exception:
                        pass
            self._sound_proc = None
            self._sound_kind = None
        cmd = [self._aplay_path, '-q']
        if self._alert_sound_device:
            cmd += ['-D', self._alert_sound_device]
        cmd.append(self._alert_sound_path)
        try:
            self._sound_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL)
            self._sound_kind = 'alert'
            self._alert_last_time = now
            # Force beacon to retrigger after the alert finishes.
            self._sound_last_start_time = None
        except Exception as exc:
            self.get_logger().warn(f'aplay launch (alert) failed: {exc}')
        return True

    def destroy_node(self) -> bool:
        try:
            if self._sound_proc is not None and self._sound_proc.poll() is None:
                self._sound_proc.terminate()
                try:
                    self._sound_proc.wait(timeout=0.5)
                except Exception:
                    self._sound_proc.kill()
            if self._hl is not None:
                self._hl.off(); self._hl.close()
            if self._bn is not None:
                self._bn.off(); self._bn.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IndicatorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
