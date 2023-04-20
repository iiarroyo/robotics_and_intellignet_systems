#!/usr/bin/env python
import numpy as np
import my_constants as constants


np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})


class DeadReckoning():
    def __init__(self):
        pass

    def calc_vals(self, miu, E, v, w, Q):
        dt = 0.1
        H = np.array([[1, 0, -dt*v*np.sin(miu[2])],
                      [0, 1, dt*v*np.cos(miu[2])],
                      [0, 0, 1]])
        E = H.dot(E).dot(H.T) + Q
        miu[0] += dt*v*np.cos(miu[2]) 
        miu[1] += dt*v*np.sin(miu[2])
        miu[2] += dt*w

        return miu, E, H

        

if __name__ == "__main__":
    a = DeadReckoning()
    miu = np.zeros((3, 1))
    E = np.zeros((3, 3))
    Q = np.array([[0.5, 0.01, 0.01],
                  [0.01, 0.5, 0.01],
                  [0.01, 0.01, 0.2]])
    v = 1
    w = 1
    dt = 0.1

    print("miu_0:\n{}".format(miu))
    print("E_0:\n{}".format(E))
    # print("AAAAAAAAAAAAAAAA{}".format(np.float(E[0][0])))
    miu, E, H = a.calc_vals(miu, E, v, w, Q)
    print("miu_1:\n{}".format(miu))
    print("E_1:\n{}".format(E))
    print("H_1:\n{}".format(H))
    a.calc_vals(miu, E, v, w, Q)
    print("miu_2:\n{}".format(miu))
    print("E_2:\n{}".format(E))
    print("H_2:\n{}".format(H))


