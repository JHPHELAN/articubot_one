#!/usr/bin/env python3
"""
Publish WiFi link metrics for every wireless interface as std_msgs/Float32
at 1 Hz. Consumed by PlotJuggler or Foxglove Studio as time-series plots.

Topics (one set per interface found in /proc/net/wireless):
  /wifi/<iface>/quality       link quality  (0..70, higher = better)
  /wifi/<iface>/signal_dbm    RSSI in dBm   (closer to 0 = better)
  /wifi/<iface>/bitrate_mbps  current TX rate, from `iw dev <iface> link`

No root required. No external ROS packages required.
"""
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

WIRELESS_PATH = '/proc/net/wireless'

# quality/level/noise are the 3rd, 4th, 5th whitespace-separated fields
# after the "iface:" token; values in /proc are floats with a trailing dot.
_PROC_RE = re.compile(
    r'^\s*(?P<iface>\S+?):\s+\S+\s+'
    r'(?P<quality>-?\d+\.?)\s+'
    r'(?P<level>-?\d+\.?)\s+'
    r'(?P<noise>-?\d+\.?)'
)
_BITRATE_RE = re.compile(r'tx bitrate:\s+([\d.]+)\s*MBit/s')


class WifiPublisher(Node):
    def __init__(self):
        super().__init__('wifi_publisher')
        self._pubs = {}  # (iface, metric) -> Publisher
        self.create_timer(1.0, self._tick)
        self.get_logger().info('wifi_publisher started (1 Hz)')

    def _pub(self, iface: str, metric: str):
        key = (iface, metric)
        p = self._pubs.get(key)
        if p is None:
            p = self.create_publisher(Float32, f'/wifi/{iface}/{metric}', 10)
            self._pubs[key] = p
        return p

    def _publish(self, iface: str, metric: str, value: float):
        msg = Float32()
        msg.data = float(value)
        self._pub(iface, metric).publish(msg)

    def _read_proc(self):
        try:
            with open(WIRELESS_PATH, 'r') as f:
                # skip 2 header lines
                lines = f.readlines()[2:]
        except OSError as e:
            self.get_logger().warning(f'{WIRELESS_PATH}: {e}')
            return {}
        result = {}
        for line in lines:
            m = _PROC_RE.match(line)
            if not m:
                continue
            iface = m.group('iface')
            quality = float(m.group('quality').rstrip('.'))
            level = float(m.group('level').rstrip('.'))
            result[iface] = {'quality': quality, 'signal_dbm': level}
        return result

    def _read_bitrate(self, iface: str):
        try:
            r = subprocess.run(
                ['iw', 'dev', iface, 'link'],
                capture_output=True, text=True, timeout=1.0
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return None
        if r.returncode != 0:
            return None
        m = _BITRATE_RE.search(r.stdout)
        return float(m.group(1)) if m else None

    def _tick(self):
        for iface, metrics in self._read_proc().items():
            for metric_name, value in metrics.items():
                self._publish(iface, metric_name, value)
            br = self._read_bitrate(iface)
            if br is not None:
                self._publish(iface, 'bitrate_mbps', br)


def main():
    rclpy.init()
    node = WifiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
