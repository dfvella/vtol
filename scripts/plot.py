#!/usr/bin/env python3

import sys
import matplotlib.pyplot as plt

try:
    filename = sys.argv[1]
except IndexError:
    print('error: data file not specified')
    exit(1)


#response = [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ]
#response = [ 0.2, 0.2, 0.2, 0.2, 0.1, 0.05, 0.02, 0.02, 0.01, 0.0 ]
#response = [ 0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] # delay <20 ms
response = [ 0.3, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] # delay ~25 ms
buffer = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

assert(abs(sum(response) - 1) < 0.001)
assert(len(response) == len(buffer))

def filter(data: float, buff: list, resp: list) -> float:
    buff.insert(0, data)
    buff.pop()

    res = 0
    for data, weight in zip(buff, resp):
        res += data * weight
    return res


tList = []
xList = []
dList = []
fList = []

with open(filename, 'r') as f:
    for line in f:
        try:
            t, x = line.split()
        except TypeError:
            print('error: failed to parse line')
            exit(1)

        if len(tList) > 1:
            d = (xList[-1] - xList[-2]) / (tList[-1] - tList[-2])
        else:
            d = 0

        tList.append(float(t))
        xList.append(float(x))
        dList.append(float(d))
        fList.append(float(filter(d, buffer, response)))

plt.plot(tList, xList, label='x')
plt.plot(tList, dList, label='dx/dt')
plt.plot(tList, fList, label='filtered')
plt.title(filename)
plt.ylabel('x')
plt.xlabel('t')
plt.legend()
plt.show()
