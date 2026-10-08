#!/usr/bin/env python
#

"""Estimates the (x, y) position of a local NM3 modem by pinging 3 remote modems at known (x, y, z) locations, 
using the acoustic ping (time-of-flight) command. The local modem's own depth (z) must be known.
Requires a local NM3 modem plus the remote modems described in the config
file.

All run parameters (serial port, remote modem addresses/locations, local
depth, speed of sound, timing) are read from a JSON config file - see
examples/trilateration_config.json for the format and an example.

Example usage:
    python -m examples.trilateration_example --config examples/trilateration_config.json
"""

import argparse
import json
import signal
import sys
import time

import serial

from nm3driver.nm3driver import Nm3
from examples.modem_ping import ModemPing


_running = True


def _handle_sigint(_signum, _frame):
    global _running
    _running = False


def load_config(config_path):
    """Loads and validates the trilateration config file. Exits with an
    error message on any missing/malformed field rather than raising."""
    try:
        with open(config_path, 'r') as config_file:
            config = json.load(config_file)
    except (OSError, json.JSONDecodeError) as e:
        print('Failed to read config file {:s}: {:s}'.format(config_path, str(e)))
        sys.exit(1)

    required_keys = ('serial_port', 'speed_of_sound', 'local_depth', 'remote_modems')
    for key in required_keys:
        if key not in config:
            print('Config file {:s} is missing required field "{:s}".'.format(config_path, key))
            sys.exit(1)

    remote_modems = config['remote_modems']
    if not isinstance(remote_modems, list) or len(remote_modems) < 3:
        print('Config field "remote_modems" must be a list of at least 3 modems.')
        sys.exit(1)

    for i, modem in enumerate(remote_modems):
        for field in ('address', 'x', 'y', 'z'):
            if field not in modem:
                print('remote_modems[{:d}] is missing required field "{:s}".'.format(i, field))
                sys.exit(1)

    config.setdefault('count', 0)
    config.setdefault('interval', 2.0)
    config.setdefault('timeout', 5.0)

    return config


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Trilateration Example. '
                    'Pings 3 or more remote modems at known locations and estimates the '
                    'local modem (x, y) position. '
                    'Example usage: python -m examples.trilateration_example '
                    '--config examples/trilateration_config.json')

    cmdline_parser.add_argument('--config', required=True,
                                help='Path to the JSON config file describing the serial port, '
                                     'remote modem addresses/locations, local depth, speed of '
                                     'sound, and timing. See examples/trilateration_config.json.')

    cmdline_args = cmdline_parser.parse_args()

    config = load_config(cmdline_args.config)

    serial_port_name = config['serial_port']
    count = config['count']
    interval = config['interval']

    # Handle Ctrl-C cleanly rather than raising KeyboardInterrupt mid-ping.
    signal.signal(signal.SIGINT, _handle_sigint)

    # Serial Port is opened with a 100ms timeout for reading - non-blocking.
    with serial.Serial(serial_port_name, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:
        nm3 = Nm3(input_stream=serial_port, output_stream=serial_port)
        modem_ping = ModemPing(nm3,
                                remote_modems=config['remote_modems'],
                                local_depth=config['local_depth'],
                                speed_of_sound=config['speed_of_sound'],
                                timeout=config['timeout'])

        print('Trilaterating against {:d} remote modems on {:s}. Press Ctrl-C to stop.'.format(
            len(config['remote_modems']), serial_port_name))

        rounds_run = 0
        while _running and (count <= 0 or rounds_run < count):
            rounds_run += 1

            timestamp_str = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())

            result, error = modem_ping.estimate_position()
            if error is not None:
                print('{:s} #{:d}: {:s} - skipping this round'.format(timestamp_str, rounds_run, error))
            else:
                x_est, y_est, ranges_m = result
                print('{:s} #{:d}: pos=({:.2f}, {:.2f}, {:.2f}) ranges={:s}'.format(
                    timestamp_str, rounds_run, x_est, y_est, config['local_depth'],
                    ', '.join('{:.2f}m'.format(r) for r in ranges_m)))

            if not _running or (count > 0 and rounds_run >= count):
                break

            # Sleep in short increments so Ctrl-C is responsive.
            sleep_until = time.time() + interval
            while _running and time.time() < sleep_until:
                time.sleep(0.05)

    print('Trilateration stopped after {:d} round(s).'.format(rounds_run))


if __name__ == '__main__':
    main()
