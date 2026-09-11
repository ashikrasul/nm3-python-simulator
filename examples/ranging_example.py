#!/usr/bin/env python
#

"""Ranges a real NM3 modem against another over a serial port, using the
acoustic ping (time-of-flight) command. Requires two physical NM3 modems.

Example usage:
    python -m examples.ranging_example --serial_port /dev/ttyUSB0 --address 2
    python -m examples.ranging_example --serial_port COM5 --address 2 --count 20 --interval 2.0
"""

import argparse
import signal
import time

import serial

from nm3driver.nm3driver import Nm3


_running = True


def _handle_sigint(signum, frame):
    global _running
    _running = False


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Ranging Example. '
                    'Pings a remote modem and reports time-of-flight and range. '
                    'Example usage: python -m examples.ranging_example')

    cmdline_parser.add_argument('--serial_port', required=True,
                                help='The local serial port connected to the NM3 modem.')

    cmdline_parser.add_argument('--address', required=True, type=int,
                                help='The address (0-255) of the remote modem to ping.')

    cmdline_parser.add_argument('--count', type=int, default=0,
                                help='Number of pings to send. 0 (default) means run until Ctrl-C.')

    cmdline_parser.add_argument('--interval', type=float, default=.50,
                                help='Seconds to wait between pings. Default 2.0.')

    cmdline_parser.add_argument('--timeout', type=float, default=5.0,
                                help='Seconds to wait for a ping response before giving up. Default 5.0.')

    cmdline_parser.add_argument('--speed_of_sound', type=float, default=1500.0,
                                help='Speed of sound in m/s used to convert time-of-flight to range. Default 1500.0.')

    cmdline_args = cmdline_parser.parse_args()

    serial_port_name = cmdline_args.serial_port
    remote_address = cmdline_args.address
    count = cmdline_args.count
    interval = cmdline_args.interval
    timeout = cmdline_args.timeout
    speed_of_sound = cmdline_args.speed_of_sound

    # Handle Ctrl-C cleanly rather than raising KeyboardInterrupt mid-ping.
    signal.signal(signal.SIGINT, _handle_sigint)

    # Serial Port is opened with a 100ms timeout for reading - non-blocking.
    with serial.Serial(serial_port_name, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
        nm3 = Nm3(input_stream=serial_port, output_stream=serial_port)

        print('Pinging address {:d} on {:s}. Press Ctrl-C to stop.'.format(remote_address, serial_port_name))

        pings_sent = 0
        while _running and (count <= 0 or pings_sent < count):
            pings_sent += 1

            send_time = time.time()
            tof = nm3.send_ping(remote_address, timeout=timeout)

            timestamp_str = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(send_time))

            if tof is None or tof < 0:
                print('{:s} #{:d}: ping to {:d} failed or timed out'.format(
                    timestamp_str, pings_sent, remote_address))
            else:
                range_m = tof * speed_of_sound
                print('{:s} #{:d}: address={:d} tof={:.6f}s range={:.2f}m'.format(
                    timestamp_str, pings_sent, remote_address, tof, range_m))

            if not _running or (count > 0 and pings_sent >= count):
                break

            # Sleep in short increments so Ctrl-C is responsive.
            sleep_until = time.time() + interval
            while _running and time.time() < sleep_until:
                time.sleep(0.05)

    print('Ranging stopped after {:d} ping(s).'.format(pings_sent))


if __name__ == '__main__':
    main()
