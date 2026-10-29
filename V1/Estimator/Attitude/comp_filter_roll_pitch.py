"""
Complementary roll/pitch Filter: 
1) motor model
    Inputs: body rates of gyro, body acce of accelero, time step
    Outputs: estimated roll & pitch
2)synthetic test bench.

Conventions: 
-> reference frame: X forward, Y left, Z up.
-> body frame: U forward, V left, W up.
Angles in radians, angular velocities in rad/s, accelerations in m/s².
All CONFIG parameters are arbitrary values to be calibrated experimentally with hardware.
"""

from dataclasses import dataclass
import math
import random

G = 9.81



@dataclass
class FilterConfig:
    integration_method = "euler_angles"  # "approx_small_angles" or "euler_angles"
    alpha: float = 0.98                         # weight of gyro (Tuning required)
    gyro_bias: tuple = (0.0, 0.0, 0.0)          # rad/s (Tuning required)
    accel_bias: tuple = (0.0, 0.0, 0.0)         # m/s² (Tuning required)
    accel_gate: float = 1.15                    # gating : ‖a‖ / g (Tuning required)
    dt_max: float = 0.05                        # gating of time steps (Tuning required)



class ComplementaryFilter:
    """Estimates roll and pitch angles based on gyro & accelerometer."""

    def __init__(self, config: FilterConfig):
        self.config = config
        self.roll = 0.0
        self.pitch = 0.0

    @staticmethod
    def accel_angles(au, av, aw):
        """Computing roll & pitch angles / Assumption: drone is at rest."""
        roll = math.atan2(av, aw)
        pitch = math.atan2(-au, math.sqrt(av**2 + aw**2))
        return roll, pitch

    def compute_gyro_approx_angles(self, gu, gv, dt):
        """Approximation small angles : drone rotative velocities ~ Euler velocities."""
        roll_gyro = self.roll + gu * dt
        pitch_gyro = self.pitch + gv * dt
        return roll_gyro, pitch_gyro

    def compute_gyro_euler_angles(self, gu, gv, gw, dt):
        """Computing Euler angles derivative based on drone rotative velocities."""
        roll_rate = gu + gv * math.sin(self.roll)*math.tan(self.pitch) + gw * math.cos(self.roll)*math.tan(self.pitch)
        pitch_rate = gv * math.cos(self.roll) - gw * math.sin(self.roll)
        roll_gyro = self.roll + roll_rate * dt
        pitch_gyro = self.pitch + pitch_rate * dt
        return roll_gyro, pitch_gyro

    def update(self, gyro, accel, dt):
        """gyro = (gu, gv, gw) rad/s ; accel = (au, av, aw) m/s² ; dt in s"""

        # Refusing update if time step unconsistent
        if dt <= 0.0 or dt > self.config.dt_max:
            return self.roll, self.pitch

        # Estimating real rotational speeds and accelerations by removing bias of sensors 
        gu, gv, gw = (g - b for g, b in zip(gyro, self.config.gyro_bias))
        au, av, aw = (a - b for a, b in zip(accel, self.config.accel_bias))

        # Computing roll and pitch from accelero
        roll_acc, pitch_acc = self.accel_angles(au, av, aw)

        # Prediction with gyro integration
        if self.config.integration_method == "approx_small_angles":
            roll_gyro, pitch_gyro = self.compute_gyro_approx_angles(gu, gv, dt)
        elif self.config.integration_method == "euler_angles":
            roll_gyro, pitch_gyro = self.compute_gyro_euler_angles(gu, gv, gw, dt)
        else:
            raise ValueError(f"Unknown gyro integration method: {self.config.integration_method}")
        

        # Accelerometer gating : checking if total acceleration close to g (rejecting influence of dynamics)
        norm_acc = math.sqrt(au * au + av * av + aw * aw)
        num_G = norm_acc / G

        # Gyro weight: alpha at rest (num_G = 1), rising linearly to 1 at the gate
        dev = min(abs(num_G - 1) / (self.config.accel_gate - 1), 1.0)
        a = self.config.alpha + (1 - self.config.alpha) * dev

        self.roll = a * roll_gyro + (1 - a) * roll_acc
        self.pitch = a * pitch_gyro + (1 - a) * pitch_acc
        
        return self.roll, self.pitch