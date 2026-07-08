#!/usr/bin/env python3
"""
Publish WiFi link metrics for every wireless interface as std_msgs/Float32.
Consumed by PlotJuggler or Foxglove Studio as time-series plots.

Data sources
------------
* Primary : ``iw dev <iface> link`` — provides ``signal:``, ``signal avg:``,
  ``tx bitrate:``, ``rx bitrate:`` and ``freq:``. ``iw`` only emits a
  ``signal:`` line when the driver actually has a fresh measurement, so it
  does not produce the spurious ``-255`` spikes that ``/proc/net/wireless``
  exhibits with some USB-dongle drivers (drivers that report dBm without
  setting the ``IW_QUAL_DBM`` flag, causing the kernel to print
  ``level - 0x100``; e.g. ``1 - 256 = -255``).
* Fallback : ``/proc/net/wireless`` — used for ``quality`` / ``noise`` (not
  reported by ``iw``) and as a fallback ``signal_dbm`` source when the
  primary is missing. Values are sanity-filtered against ``SIGNAL_RANGE_DBM``
  so obviously-invalid samples are dropped instead of published.

Published topics (one set per interface found in ``/proc/net/wireless``)
-----------------------------------------------------------------------
* ``/wifi/<iface>/quality``          link quality (higher = better)
* ``/wifi/<iface>/signal_dbm``       instantaneous RSSI  (closer to 0 = better)
* ``/wifi/<iface>/signal_avg_dbm``   driver-smoothed RSSI  (when reported)
* ``/wifi/<iface>/noise_dbm``        noise floor           (when reported)
* ``/wifi/<iface>/bitrate_mbps``     current TX rate       (legacy alias)
* ``/wifi/<iface>/tx_bitrate_mbps``  current TX rate
* ``/wifi/<iface>/rx_bitrate_mbps``  current RX rate
* ``/wifi/<iface>/frequency_mhz``    associated channel frequency

Parameters
----------
* ``publish_hz``    (double, default ``1.0``) — publish rate.
* ``prefer_iw``     (bool,   default ``True``) — try ``iw`` before ``/proc``.
* ``reject_warn_period_sec`` (double, default ``10.0``) — throttle for
  "rejected sample" warnings, per (iface, metric).

No root required. No external ROS packages required.
"""
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

WIRELESS_PATH = '/proc/net/wireless'

# Plausibility ranges. Samples outside these are dropped, not published.
SIGNAL_RANGE_DBM = (-110.0, -10.0)   # consumer WiFi RSSI
QUALITY_RANGE = (0.0, 100.0)         # /proc reports x/70 or x/100 depending on driver
BITRATE_RANGE_MBPS = (0.1, 15000.0)  # accommodate Wi-Fi 6/7 rates
FREQ_RANGE_MHZ = (2000.0, 7500.0)    # 2.4/5/6 GHz bands

# quality/level/noise are the 3rd, 4th, 5th whitespace-separated fields
# after the "iface:" token; values in /proc are floats with a trailing dot.
_PROC_RE = re.compile(
    r'^\s*(?P<iface>\S+?):\s+\S+\s+'
    r'(?P<quality>-?\d+\.?)\s+'
    r'(?P<level>-?\d+\.?)\s+'
    r'(?P<noise>-?\d+\.?)'
)

# Well-known "no measurement" sentinels produced by the kernel's wext-proc
# formatter when the driver reports dBm without setting IW_QUAL_DBM: the
# formatter prints ``raw - 0x100``. A raw driver value of 0 yields -256 and
# 1 yields -255. Most consumer WiFi drivers never measure noise and so
# report 0 on every read → -256 shows up on every tick. Treat these as
# "not reported" and drop them silently rather than warning on every tick.
#
# 0.0 is also included: 0 dBm (== 1 mW at the receiver) is physically
# impossible for consumer WiFi RSSI/noise, so any source reporting exactly
# 0.0 is a "no fresh sample" placeholder from the driver, not a real value.
# Some USB dongles (rtl88x2bu, mt76xxu, etc.) emit this between real
# measurements and it spams the reject-warning logic once per tick.
_PROC_SENTINELS = {0.0, -255.0, -256.0}

# `iw dev <iface> link` field parsers. Multiline so ^ anchors each line and
# "signal avg:" doesn't accidentally satisfy the "signal:" regex.
_IW_SIGNAL_RE     = re.compile(r'^\s*signal:\s*(-?\d+)\s*dBm',       re.MULTILINE)
_IW_SIGNAL_AVG_RE = re.compile(r'^\s*signal avg:\s*(-?\d+)\s*dBm',   re.MULTILINE)
_IW_TX_RATE_RE    = re.compile(r'^\s*tx bitrate:\s*([\d.]+)\s*MBit/s', re.MULTILINE)
_IW_RX_RATE_RE    = re.compile(r'^\s*rx bitrate:\s*([\d.]+)\s*MBit/s', re.MULTILINE)
_IW_FREQ_RE       = re.compile(r'^\s*freq:\s*(\d+)',                 re.MULTILINE)


def _in_range(value, lo_hi):
    lo, hi = lo_hi
    return lo <= value <= hi


class WifiPublisher(Node):
    def __init__(self):
        super().__init__('wifi_publisher')

        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('prefer_iw', True)
        self.declare_parameter('reject_warn_period_sec', 10.0)

        self._publish_hz = float(self.get_parameter('publish_hz').value)
        self._prefer_iw = bool(self.get_parameter('prefer_iw').value)
        self._warn_period = float(
            self.get_parameter('reject_warn_period_sec').value)

        # Runtime state.
        self._pubs = {}          # (iface, metric) -> Publisher
        self._last_warn = {}     # (iface, metric) -> monotonic seconds
        self._iw_missing_logged = False

        period = 1.0 / self._publish_hz if self._publish_hz > 0 else 1.0
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'wifi_publisher started ({self._publish_hz:.2f} Hz, '
            f'prefer_iw={self._prefer_iw})'
        )

    # ---- publishing helpers -------------------------------------------------
    def _pub(self, iface: str, metric: str):
        key = (iface, metric)
        p = self._pubs.get(key)
        if p is None:
            p = self.create_publisher(Float32, f'/wifi/{iface}/{metric}', 10)
            self._pubs[key] = p
        return p

    def _publish(self, iface: str, metric: str, value: float, valid_range):
        """Publish ``value`` on ``/wifi/<iface>/<metric>`` iff in range."""
        if value is None:
            return
        try:
            fv = float(value)
        except (TypeError, ValueError):
            return
        if valid_range is not None and not _in_range(fv, valid_range):
            self._warn_rejected(iface, metric, fv, valid_range)
            return
        msg = Float32()
        msg.data = fv
        self._pub(iface, metric).publish(msg)

    def _warn_rejected(self, iface, metric, value, valid_range):
        key = (iface, metric)
        now = time.monotonic()
        last = self._last_warn.get(key, 0.0)
        if now - last < self._warn_period:
            return
        self._last_warn[key] = now
        self.get_logger().warning(
            f'{iface}/{metric}: rejected sample {value:g} '
            f'(outside {valid_range[0]:g}..{valid_range[1]:g}) — '
            'likely stale/invalid driver reading'
        )

    # ---- data sources -------------------------------------------------------
    def _read_proc(self):
        """Return ``{iface: {'quality': q, 'signal_dbm': l, 'noise_dbm': n}}``."""
        try:
            with open(WIRELESS_PATH, 'r') as f:
                lines = f.readlines()[2:]  # skip 2 header lines
        except OSError as e:
            self.get_logger().warning(f'{WIRELESS_PATH}: {e}')
            return {}
        result = {}
        for line in lines:
            m = _PROC_RE.match(line)
            if not m:
                continue
            iface = m.group('iface')
            metrics = {'quality': float(m.group('quality').rstrip('.'))}
            # Signal / noise: drop the well-known kernel sentinels silently
            # so we don't spam warnings for what is normal driver behaviour.
            level = float(m.group('level').rstrip('.'))
            if level not in _PROC_SENTINELS:
                metrics['signal_dbm'] = level
            noise = float(m.group('noise').rstrip('.'))
            if noise not in _PROC_SENTINELS:
                metrics['noise_dbm'] = noise
            result[iface] = metrics
        return result

    def _read_iw(self, iface: str):
        """Return a dict of iw-derived metrics, or ``{}`` if unavailable."""
        try:
            r = subprocess.run(
                ['iw', 'dev', iface, 'link'],
                capture_output=True, text=True, timeout=1.0
            )
        except FileNotFoundError:
            if not self._iw_missing_logged:
                self.get_logger().warning(
                    "'iw' not found — falling back to /proc/net/wireless "
                    "for signal (may show spurious -255 spikes on some USB "
                    "dongles). Install with: sudo apt install iw"
                )
                self._iw_missing_logged = True
            return {}
        except subprocess.TimeoutExpired:
            return {}
        if r.returncode != 0:
            return {}

        out = r.stdout
        result = {}

        # Same 0-dBm suppression as the /proc path: some USB drivers emit
        # 'signal: 0 dBm' when they have no fresh sample. Drop silently
        # instead of publishing a bogus value or warning every tick.
        m = _IW_SIGNAL_RE.search(out)
        if m:
            val = float(m.group(1))
            if val not in _PROC_SENTINELS:
                result['signal_dbm'] = val
        m = _IW_SIGNAL_AVG_RE.search(out)
        if m:
            val = float(m.group(1))
            if val not in _PROC_SENTINELS:
                result['signal_avg_dbm'] = val
        m = _IW_TX_RATE_RE.search(out)
        if m:
            result['tx_bitrate_mbps'] = float(m.group(1))
        m = _IW_RX_RATE_RE.search(out)
        if m:
            result['rx_bitrate_mbps'] = float(m.group(1))
        m = _IW_FREQ_RE.search(out)
        if m:
            result['frequency_mhz'] = float(m.group(1))
        return result

    # ---- main loop ----------------------------------------------------------
    def _tick(self):
        proc_data = self._read_proc()
        for iface, proc_metrics in proc_data.items():
            iw_metrics = self._read_iw(iface) if self._prefer_iw else {}

            # Signal: iw preferred (no -255 sentinels), /proc as fallback.
            signal = iw_metrics.get('signal_dbm', proc_metrics.get('signal_dbm'))
            self._publish(iface, 'signal_dbm', signal, SIGNAL_RANGE_DBM)

            # Averaged signal is iw-only; publish only when the driver reports it.
            self._publish(iface, 'signal_avg_dbm',
                          iw_metrics.get('signal_avg_dbm'), SIGNAL_RANGE_DBM)

            # Quality/noise come from /proc only.
            self._publish(iface, 'quality',
                          proc_metrics.get('quality'), QUALITY_RANGE)
            self._publish(iface, 'noise_dbm',
                          proc_metrics.get('noise_dbm'), SIGNAL_RANGE_DBM)

            # Bitrates + frequency come from iw.
            tx = iw_metrics.get('tx_bitrate_mbps')
            self._publish(iface, 'tx_bitrate_mbps', tx, BITRATE_RANGE_MBPS)
            # Legacy alias for existing PlotJuggler layouts.
            self._publish(iface, 'bitrate_mbps', tx, BITRATE_RANGE_MBPS)
            self._publish(iface, 'rx_bitrate_mbps',
                          iw_metrics.get('rx_bitrate_mbps'),
                          BITRATE_RANGE_MBPS)
            self._publish(iface, 'frequency_mhz',
                          iw_metrics.get('frequency_mhz'), FREQ_RANGE_MHZ)


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
