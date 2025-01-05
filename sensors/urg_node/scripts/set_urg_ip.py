#!/usr/bin/env python3

# Copyright (c) 2014 Unbounded Robotics Inc.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Unbounded Robotics Inc. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNBOUNDED ROBOTICS INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Change the IP address of a Hokuyo URG Laser."""

import argparse
import socket


def parse_and_validate_ipv4(argument, name):
    """Each address must have 4."""
    if len(argument.split('.')) != 4:
        print('Invalid %s, must be of the form xxx.yyy.zzz.www' % name)
        exit(-1)
    parsed = ''
    for x in argument.split('.'):
        if len(x) > 3:
            print('Invalid %s, must be of the form xxx.yyy.zzz.www' % name)
            exit(-1)
        while len(x) < 3:
            x = '0' + x
        parsed += x
    return parsed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('new_ip', help='The desired IP address for the laser')
    parser.add_argument('new_gw', help='The desired gateway for the laser')
    parser.add_argument('--nm', help='The desired netmask for the laser', default='255.255.255.0')
    parser.add_argument('--ip', help='The current IP address of the laser', default='192.168.0.10')
    args = parser.parse_args()

    # Packet starts with $IP, ends with \x0a, contains:
    #   12 character IP
    #   12 character netmask
    #   12 character gateway
    ip = parse_and_validate_ipv4(args.new_ip, 'IP address')
    gw = parse_and_validate_ipv4(args.new_gw, 'gateway address')
    nm = parse_and_validate_ipv4(args.nm, 'netmask')
    msg = '$IP' + ip + nm + gw + '\x0a'

    print('Connecting to %s' % args.ip)
    sock = socket.socket()
    sock.connect((args.ip, 10940))

    print('Updating settings')
    sock.send(msg)
    try:
        sock.settimeout(5)
        returned = sock.recv(40)
    except socket.timeout:
        print('Laser did not return any packet, is probably not updated.')
        exit(-1)
    if msg != returned:
        print('Laser does not appear to have updated')
        exit(-1)
    print('Done updating, cycle power on laser')
