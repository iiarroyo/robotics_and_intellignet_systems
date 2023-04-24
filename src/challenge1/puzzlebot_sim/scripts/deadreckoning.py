#!/usr/bin/env python
import numpy as np
import my_constants as constants


np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})


class DeadReckoning():
    def __init__(self):
        pass

    def calc_vals(self, miu, E, v, w, Q, dt, wr, wl):
        kr = 1.1
        kl = 1.1
        wheel_cov = np.array([[kr*abs(wr), 0],
                              [0, kl*abs(wl)]])
        nabla_w = (0.5)*constants.r*dt*np.array([[np.cos(miu[2]), np.cos(miu[2])],
                                                 [np.sin(miu[2]), np.sin(miu[2])],
                                                 [2/constants.L, -2/constants.L]])
        
        Q = nabla_w.dot(wheel_cov).dot(nabla_w.T)
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
    miu = np.array([[0.0],
                    [1.0],
                    [10.0]])
    E = np.zeros((3, 3))
    # Q = np.array([[0.5, 0.01, 0.01],
    #               [0.01, 0.5, 0.01],
    #               [0.01, 0.01, 0.2]])
    Q = np.array([[0.05, 0.001, 0.001],
                  [0.001, 0.05, 0.001],
                  [0.001, 0.001, 0.02]])
    v = 1
    w = 1
    dt = 0.1

    a.calc_vals(miu, E, v, w, Q, dt, wr=1, wl=1)


