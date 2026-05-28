# -*- coding: utf-8 -*-
import numpy as np

class MahonyAHRS:
    """内置高性能 9轴 Mahony 姿态融合算法引擎"""
    def __init__(self, kp=0.5, ki=0.0):
        self.kp = kp  
        self.ki = ki  
        self.q = [1.0, 0.0, 0.0, 0.0]  
        self.e_int = [0.0, 0.0, 0.0]   

    def update_9dof(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
        q0, q1, q2, q3 = self.q
        
        norm = np.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0: return
        ax, ay, az = ax/norm, ay/norm, az/norm
        
        norm = np.sqrt(mx*mx + my*my + mz*mz)
        if norm == 0: return
        mx, my, mz = mx/norm, my/norm, mz/norm
        
        hx = mx*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2)
        hy = 2*mx*(q1*q2 + q0*q3) + my*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2*mz*(q2*q3 - q0*q1)
        bx = np.sqrt(hx*hx + hy*hy)
        bz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + mz*(q0*q0 - q1*q1 - q2*q2 + q3*q3)
        
        vx = 2.0 * (q1*q3 - q0*q2)
        vy = 2.0 * (q0*q1 + q2*q3)
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
        
        wx = bx*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2.0*bz*(q1*q3 - q0*q2)
        wy = 2.0*bx*(q1*q2 - q0*q3) + 2.0*bz*(q0*q1 + q2*q3)
        wz = 2.0*bx*(q1*q3 + q0*q2) + bz*(q0*q0 - q1*q1 - q2*q2 + q3*q3)
        
        ex = (ay*vz - az*vy) + (my*wz - mz*wy)
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz)
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx)
        
        if self.ki > 0:
            self.e_int[0] += ex * dt
            self.e_int[1] += ey * dt
            self.e_int[2] += ez * dt
        else:
            self.e_int = [0.0, 0.0, 0.0]
            
        gx += self.kp * ex + self.ki * self.e_int[0]
        gy += self.kp * ey + self.ki * self.e_int[1]
        gz += self.kp * ez + self.ki * self.e_int[2]
        
        q0 += (-q1*gx - q2*gy - q3*gz) * (0.5 * dt)
        q1 += (q0*gx + q2*gz - q3*gy) * (0.5 * dt)
        q2 += (q0*gy - q1*gz + q3*gx) * (0.5 * dt)
        q3 += (q0*gz + q1*gy - q2*gx) * (0.5 * dt)
        
        norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = [q0/norm, q1/norm, q2/norm, q3/norm]

    def get_euler(self):
        q0, q1, q2, q3 = self.q
        roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1.0, 1.0))
        yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)