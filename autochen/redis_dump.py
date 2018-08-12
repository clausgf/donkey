#!/usr/bin/env python3
"""
Scripts to dump information from redis.

Usage:
    redis_dump.py [--host=HOSTNAME] [--port=PORT] [--refresh=INTERVAL]

Options:
    -h --help           Show this screen.
    --host=<HOSTNAME>   Host name of the redis server [default: picam2].
    --port=<PORT>       Port number of the redis server [default: 6379].
"""
import time
import redis
from curses import *
from docopt import docopt

import donkeycar as dk


global host
global port
global refresh_interval

global default_keys
default_keys = ['user/mode', 'user/angle', 'user/throttle',
                'pilot/angle', 'pilot/throttle',
                'angle', 'throttle', 'run_pilot', 'recording']

def main(stdscr):
    halfdelay(3)
    r = redis.StrictRedis(host, port)

    while True:
        stdscr.clear()
        header = "Redis {}:{}".format(host, port)
        stdscr.addstr(0, 0, header.ljust(stdscr.getmaxyx()[1]), A_REVERSE)

        keys = sorted(r.keys())
        values = r.mget(keys)
        for i,key in enumerate(keys):
            k = key.decode('utf-8')
            v = values[i]
            if v: v = v.decode('utf-8')
            s = '{:20} = {}'.format(k, v)
            y = i+2
            if y < stdscr.getmaxyx()[0]:
                stdscr.addstr(y, 0, s[0:stdscr.getmaxyx()[1]])

        stdscr.refresh()
        key = stdscr.getch()
        if key != ERR:
            break


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    host = args['--host']
    port = args['--port']

    wrapper(main)
