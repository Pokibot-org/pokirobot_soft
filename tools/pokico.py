#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
from random import random

ROBOT_L = 150.0
INTERVAL = 0.01
PLANE_VMAX = 1000.0 # 1000mm/s
PLANE_FACTOR = 0.007 * PLANE_VMAX
PLANE_RAMP = 2.0 * PLANE_VMAX * INTERVAL # 1/seconds to reach vmax
ANGLE_VMAX = 2*math.pi # ~1tr/s
ANGLE_FACTOR = 0.5 * ANGLE_VMAX
ANGLE_RAMP = 2.0 * ANGLE_VMAX * INTERVAL # 1/seconds to reach vmax


class Pos:
    def __init__(self, x, y, a):
        self.x = x
        self.y = y
        self.a = a
    def __repr__(self):
        return f"{self.__class__.__name__}(x={self.x}, y={self.y}, a={self.a})"
    @classmethod
    def diff(cls, ter, ori):
        return Pos(ter.x-ori.x, ter.y-ori.y, ter.a-ori.a)

class Vel:
    def __init__(self, vx, vy, w):
        self.vx = vx
        self.vy = vy
        self.w = w
    def __repr__(self):
        return f"{self.__class__.__name__}(vx={self.vx}, vy={self.vy}, w={self.w})"

class Omni:
    def __init__(self, v1, v2, v3):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
    def __repr__(self):
        return f"{self.__class__.__name__}(v1={self.v1}, v2={self.v2}, v3={self.v3})"

class Robot:
    def __init__(self):
        self.pos = Pos(0.0, 0.0, 0.0)
        self.wvel = Vel(0.0, 0.0, 0.0)
        self.lvel = Vel(0.0, 0.0, 0.0)
        self.omni = Omni(0.0, 0.0, 0.0)


def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

def vel_cap(vel):
    v_plane = math.sqrt(vel.vx*vel.vx + vel.vy*vel.vy)
    v_clamped = clamp(v_plane, -PLANE_VMAX, PLANE_VMAX)
    if v_plane > 0.0:
        factor = v_clamped/v_plane
    else:
        factor = 0.0
    return Vel(
            factor * vel.vx,
            factor * vel.vy,
            clamp(vel.w, -ANGLE_VMAX, ANGLE_VMAX)
            )

def vel_ramp(vel, prev):
    # plane velocity caping
    v_plane = math.sqrt(vel.vx*vel.vx + vel.vy*vel.vy)
    prev_plane = math.sqrt(prev.vx*prev.vx + prev.vy*prev.vy)
    plane_max = prev_plane + PLANE_RAMP
    if v_plane > plane_max:
        plane_factor = plane_max / v_plane
    else:
        plane_factor = 1.0
    # angle capping
    v_w = abs(vel.w)
    w_max = abs(prev.w) + ANGLE_RAMP
    if v_w > w_max:
        w_factor = w_max / v_w
    else:
        w_factor = 1.0
    return Vel(
            plane_factor * vel.vx,
            plane_factor * vel.vy,
            w_factor * vel.w
            )

def world_vel_from_delta(delta, prev_vel):
    wvel = vel_cap(vel_ramp(Vel(
            PLANE_FACTOR*delta.x,
            PLANE_FACTOR*delta.y,
            ANGLE_FACTOR*delta.a), prev_vel))
    return wvel

def local_vel_from_world(pos, world_vel):
    return Vel(
            math.cos(pos.a) * world_vel.vx + math.sin(pos.a) * world_vel.vy,
            -math.sin(pos.a) * world_vel.vx + math.cos(pos.a) * world_vel.vy,
            world_vel.w)

def world_vel_from_local(pos, local_vel):
    return Vel(
            math.cos(pos.a) * local_vel.vx - math.sin(pos.a) * local_vel.vy,
            math.sin(pos.a) * local_vel.vx + math.cos(pos.a) * local_vel.vy,
            local_vel.w)

def omni_from_vel(vel):
    return Omni(
            -vel.vx / 2.0 - math.sqrt(3.0) * vel.vy/2.0 + ROBOT_L * vel.w,
            vel.vx + ROBOT_L * vel.w,
            -vel.vx / 2.0 + math.sqrt(3.0) * vel.vy / 2.0 + ROBOT_L * vel.w
            )

def vel_from_omni(omni):
    return Vel(
            (2.0 * omni.v2 - omni.v1 - omni.v3) / 3,
            (math.sqrt(3.0) * omni.v3 - math.sqrt(3.0) * omni.v1) / 3,
            (omni.v1 + omni.v2 + omni.v3) / (3 * ROBOT_L))

def task(target, limit=100.0):
    robot = Robot()
    i = 0
    t = 0
    data = []
    _target = target
    while t < limit:
        # noise
        #_target = target
        #_target.x += random() * 10.0 - 5.0
        #_target.y += random() * 10.0 - 5.0
        #_target.a += random() * 0.05 * 2 * math.pi - 0.05*math.pi
        #update pos
        m_lvel = vel_from_omni(robot.omni)
        m_wvel = world_vel_from_local(robot.pos, m_lvel)
        new_pos = Pos(
                robot.pos.x + m_wvel.vx*INTERVAL,
                robot.pos.y + m_wvel.vy*INTERVAL,
                robot.pos.a + m_wvel.w*INTERVAL)
        robot.pos = new_pos
        if i == 0:
            # new corrected speed
            delta = Pos.diff(_target, robot.pos)
            robot.wvel = world_vel_from_delta(delta, robot.wvel)
            robot.lvel = local_vel_from_world(robot.pos, robot.wvel)
            robot.omni = omni_from_vel(robot.lvel)
        line = [t,
                target.x,target.y,target.a,
                robot.pos.x,robot.pos.y,robot.pos.a,
                robot.wvel.vx,robot.wvel.vy,robot.wvel.w,
                robot.lvel.vx,robot.lvel.vy,robot.lvel.w,
                robot.omni.v1,robot.omni.v2,robot.omni.v3]
        data.append(line)
        t += INTERVAL
        #i = (i+1) % 10
    return data


def control_print(data):
    print("t,dest.x,dest.y,dest.a,pos.x,pos.y,pos.a,wv.vx,wv.vy,wv.w,lv.vx,lv.vy,lv.w,m.v1,m.v2,m.v3,")
    for l in data:
        print(",".join(map(lambda x: str(x), l)))

def control_plot(data):
    arr = np.array(data)
    t = arr[:,0]
    tx = arr[:,1]
    ty = arr[:,2]
    ta = arr[:,3]
    posx = arr[:,4]
    posy = arr[:,5]
    posa = arr[:,6]
    wv_x = arr[:,7]
    wv_y = arr[:,8]
    wv_w = arr[:,9]
    rv_x = arr[:,10]
    rv_y = arr[:,11]
    rv_w = arr[:,12]
    mv_1 = arr[:,13]
    mv_2 = arr[:,14]
    mv_3 = arr[:,15]
    f, (xy, pos, wv, rv, mv) = plt.subplots(5, 1)
    # xy
    xy.plot(posx, posy)
    xy.set_title("XY position")
    # pos
    pos.plot(t, posx, 'r-', label='x')
    pos.plot(t, posy, 'g-', label='y')
    pos_ax = pos.twinx()
    pos_ax.plot(t, posa, 'b-', label='a')
    pos.set_title("position")
    pos.legend()
    # wv
    wv.plot(t, wv_x, 'r-', label='vx')
    wv.plot(t, wv_y, 'g-', label='vy')
    wv_ax = wv.twinx()
    wv_ax.plot(t, wv_w, 'b-', label='w')
    wv.set_title("velocity (world)")
    wv.legend()
    # rv
    rv.plot(t, rv_x, 'r-', label='vx')
    rv.plot(t, rv_y, 'g-', label='vy')
    rv_ax = rv.twinx()
    rv_ax.plot(t, rv_w, 'b-', label='w')
    rv.set_title("velocity (robot)")
    rv.legend()
    # mv
    mv.plot(t, mv_1, 'r-', label='v1')
    mv.plot(t, mv_2, 'g-', label='v2')
    mv.plot(t, mv_3, 'b-', label='v3')
    mv.set_title("velocity (motor)")
    mv.legend()
    # show
    plt.draw()
    plt.show()

if __name__ == "__main__":
    target = Pos(4000.0, 2000.0, 3.0*2.0*math.pi)
    data = task(target, 10)
    #control_print(data)
    control_plot(data)
