import math
import time
import sys
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_xlim3d([-1.2, 1.2])
ax.set_ylim3d([-1.2, 1.2])
ax.set_zlim3d([-1.2, 1.2])

(lineA,) = ax.plot([0], [0], [0], label="a")
(lineB,) = ax.plot([0], [0], [0], label="b")
(lineC,) = ax.plot([0], [0], [0], label="c")
ax.legend()


def update(xyz):
    [x, y, z] = xyz
    lineA.set_data_3d(np.array([[0, x[0]], [0, x[1]], [0, x[2]]]))
    lineB.set_data_3d(np.array([[0, y[0]], [0, y[1]], [0, y[2]]]))
    lineC.set_data_3d(np.array([[0, z[0]], [0, z[1]], [0, z[2]]]))


dataFile = "data.txt" if len(sys.argv) <= 1 else sys.argv[1]
print(f"data file {dataFile}")


def openFile(f):
    if f.startswith("/dev/cu."):
        return serial.Serial(port=f, baudrate=115200, timeout=0)
    else:
        return open(f)


def frames():
    last = "##RR##|1,0,0|0,1,0|0,0,1"
    mid = bytes([])
    with openFile(dataFile) as src:
        while True:
            l = src.readline()
            if len(l) > 0:
                if type(l) is bytes:
                    if l[-1] != 10:
                        mid = mid + l
                        continue
                    else:
                        l = mid + l
                        mid = bytes([])
                    try:
                        l = l.decode()
                    except UnicodeDecodeError:
                        continue
                if l.startswith("##RR##|"):
                    last = l
            else:
                try:
                    d = [
                        list(map(lambda x: float(x), v.split(",")))
                        for v in last.split("|")[1:]
                    ]
                except ValueError:
                    print(f"line: {last}")
                else:
                    yield d


ani = animation.FuncAnimation(
    fig, update, frames, interval=200, blit=False, cache_frame_data=False
)
plt.show()
