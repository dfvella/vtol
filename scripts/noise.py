#!/usr/bin/env python3

# script for tuning FIR filter response
# usage: ./noise.py <target data> <angle data>

import sys
import matplotlib.pyplot as plt

try:
    targetDataFile = sys.argv[1]
    angleDataFile = sys.argv[2]
except IndexError:
    print('usage: ./noise.py <target data> <angle data>')
    exit(1)

BUFFER_SIZE = 10

#targetResponse = [ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
#targetResponse = [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ]
#targetResponse = [ 0.2, 0.2, 0.2, 0.2, 0.1, 0.05, 0.02, 0.02, 0.01, 0.0 ]
#targetResponse = [ 0.2, 0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0 ]
targetResponse = [ 0.3, 0.2, 0.2, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0 ]
#targetResponse = [ 0.3, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
#targetResponse = [ 0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
#targetResponse = [ 0.4, 0.4, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
#targetResponse = [ 0.5, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]


#dErrorResponse = [ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
dErrorResponse = [ 0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] # delay <20 ms
#dErrorResponse = [ 0.3, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] # delay ~25 ms


targetBuffer = [ 0.0 for _ in range(BUFFER_SIZE) ]
dErrorBuffer = [ 0.0 for _ in range(BUFFER_SIZE) ]

assert(abs(sum(targetResponse) - 1) < 0.001)
assert(len(targetResponse) == BUFFER_SIZE)

assert(abs(sum(dErrorResponse) - 1) < 0.001)
assert(len(dErrorResponse) == BUFFER_SIZE)


def filter(data: float, buff: list, resp: list) -> float:
    buff.insert(0, data)
    buff.pop()

    res = 0
    for data, weight in zip(buff, resp):
        res += data * weight
    return res


timeData = []
angleData = []
targetData = []

errorFiltered = []
dErrorFiltered = []

errorRaw = []
dErrorRaw = []

with open(angleDataFile, 'r') as f:
    for line in f:
        try:
            t, angle = line.split()
            timeData.append(float(t))
            angleData.append(float(angle))
        except TypeError:
            print('error: failed to parse line')
            exit(1)

with open(targetDataFile, 'r') as f:
    for line in f:
        try:
            _, target = line.split()
            targetData.append(float(target))
        except TypeError:
            print('error: failed to parse line')
            exit(1)

for time, angle, target in zip(timeData, angleData, targetData):
    errorRaw.append(target - angle)

    if len(errorRaw) > 1:
        dError = (errorRaw[-1] - errorRaw[-2]) / (timeData[-1] - timeData[-2])
    else:
        dError = 0
    dErrorRaw.append(dError)

    target = filter(target, targetBuffer, targetResponse)
    errorFiltered.append(target - angle)

    if len(errorFiltered) > 1:
        dError = (errorFiltered[-1] - errorFiltered[-2]) / (timeData[-1] - timeData[-2])
    else:
        dError = 0

    fdError = filter(dError, dErrorBuffer, dErrorResponse)
    dErrorFiltered.append(fdError)


#plt.plot(timeData, targetData, label='target')
#plt.plot(timeData, angleData, label='angle')

#plt.plot(timeData, errorFiltered, label='error')
plt.plot(timeData, dErrorFiltered, label='de/dt')
#plt.plot(timeData, errorRaw, label='raw')
plt.plot(timeData, dErrorRaw, label='de/dt raw')

plt.title(f'{angleDataFile} {targetDataFile}')
plt.ylabel('x')
plt.xlabel('t')
plt.legend()
plt.show()
