#!/usr/bin/env python
#

"""ModemPing: pings a set of remote NM3 modems at known (x, y, z) locations
and estimates the (x, y) position of the local modem, given its known depth.

Wraps an already-connected nm3driver.nm3driver.Nm3 instance - see
examples/trilateration_example.py for a runnable example.
"""

import math

import numpy


class ModemPing:
    """Ranges a local NM3 modem against 3 or more remote modems at known
    locations, and estimates the local modem's (x, y) position."""

    def __init__(self, nm3, remote_modems, local_depth, speed_of_sound, timeout=5.0):
        """Constructor.
        nm3: a connected nm3driver.nm3driver.Nm3 instance.
        remote_modems: list of dicts, each with 'address', 'x', 'y', 'z'. Must
            contain at least 3 entries.
        local_depth: known z of the local modem.
        speed_of_sound: speed of sound in m/s, used to convert time-of-flight to range.
        timeout: seconds to wait for each ping response before giving up.
        """
        if len(remote_modems) < 3:
            raise ValueError('remote_modems must contain at least 3 entries.')

        self._nm3 = nm3
        self._remote_modems = remote_modems
        self._local_depth = local_depth
        self._speed_of_sound = speed_of_sound
        self._timeout = timeout

    def ping_address(self, address):
        """Pings a single remote address. Returns the range in metres, or
        None if the ping failed or timed out."""
        tof = self._nm3.send_ping(address, timeout=self._timeout)
        if tof is None or tof < 0:
            return None
        return tof * self._speed_of_sound

    def estimate_position(self):
        """Pings each configured remote modem once and, if all pings succeed
        and the geometry is valid, estimates the local (x, y) position.

        Returns a tuple (result, error):
        - On success: ((x, y, ranges_m), None) where ranges_m is the list of
          slant ranges (metres) to each remote modem, in the same order as
          remote_modems.
        - On failure: (None, error_message) describing why no estimate could
          be made this round.
        """
        ranges_m = []
        for modem in self._remote_modems:
            range_m = self.ping_address(modem['address'])
            if range_m is None:
                return None, 'ping to address {:d} failed or timed out'.format(modem['address'])
            ranges_m.append(range_m)

        horizontal_ranges = []
        for modem, range_m in zip(self._remote_modems, ranges_m):
            dz = modem['z'] - self._local_depth
            if range_m < abs(dz):
                return None, 'range {:.2f}m to address {:d} is shorter than the depth separation {:.2f}m'.format(
                    range_m, modem['address'], abs(dz))
            horizontal_ranges.append(math.sqrt(range_m ** 2 - dz ** 2))

        anchors = [(modem['x'], modem['y']) for modem in self._remote_modems]
        x_est, y_est = self._solve_xy(anchors, horizontal_ranges)
        return (x_est, y_est, ranges_m), None

    @staticmethod
    def _solve_xy(anchors, horizontal_ranges):
        """Estimates an (x, y) position from 3 or more anchor (x, y)
        positions and the corresponding horizontal (x,y-plane) ranges to
        each anchor, using linearized trilateration solved by least squares.

        anchors: list of (x, y) tuples, length >= 3.
        horizontal_ranges: list of floats, same length as anchors.

        Returns (x, y).
        """
        x0, y0 = anchors[0]
        d0 = horizontal_ranges[0]

        a_rows = []
        b_rows = []
        for (xi, yi), di in zip(anchors[1:], horizontal_ranges[1:]):
            a_rows.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
            b_rows.append(d0 ** 2 - di ** 2 - x0 ** 2 + xi ** 2 - y0 ** 2 + yi ** 2)

        a_matrix = numpy.array(a_rows)
        b_vector = numpy.array(b_rows)

        solution = numpy.linalg.lstsq(a_matrix, b_vector, rcond=None)[0]
        return float(solution[0]), float(solution[1])
