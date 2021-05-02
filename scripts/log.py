#!/usr/bin/env python3

# A script that redirects Arduino serial output to a file with time stamps

import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
READ_TIME = 10

if len(sys.argv) > 1:
    LOG_FILE = sys.argv[1]
else:
    LOG_FILE = 'serial.log'

print(f'opening serial port {SERIAL_PORT} ...')

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

print(f'waiting for imu calibration ...')

while True:
    line = ser.readline()

    try:
        line = line.decode('utf-8')

        if 'setup complete' in line:
            break

    except UnicodeDecodeError:
        print('warning: failed to decode line')

print(f'redirecting {SERIAL_PORT} to {LOG_FILE} ...')

with open(LOG_FILE, 'w') as f:
    start = time.time()

    while time.time() - start < READ_TIME:
        line = ser.readline()

        try:
            line = line.decode('utf-8')
            f.write(f'{time.time() - start} {line}')

        except UnicodeDecodeError:
            print('Failed to decode line')

ser.close()

reads = 0
with open(LOG_FILE, 'r') as f:
    for lines in f:
        reads += 1

print(f'Read frequency: {int(reads / READ_TIME)} Hz')
