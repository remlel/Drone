"""
Complementary roll/pitch Filter: 
1) motor model
2)synthetic test bench.

Convention: body frame with X forward, Y left, Z up.
Angles in radians, angular velocities in rad/s, accelerations in m/s².
All CONFIG parameters are arbitrary values to be calibrated experimentally with hardware.
"""

from dataclasses import dataclass
import math
import random
import matplotlib.pyplot as plt
import numpy as np

G = 9.81



@dataclass
class FilterConfig:
    integration_method = "euler_angles"  # "approx_small_angles" or "euler_angles"
    alpha: float = 0.98                         # weight of gyro (Tuning required)
    gyro_bias: tuple = (0.0, 0.0, 0.0)          # rad/s (Tuning required)
    accel_bias: tuple = (0.0, 0.0, 0.0)         # m/s² (Tuning required)
    accel_gate: float = 0.15                    # gating : |‖a‖ - g| / g (Tuning required)
    dt_max: float = 0.05                        # gating of time steps (Tuning required)



class ComplementaryFilter:
    """Estimates roll and pitch angles based on gyro & accelerometer."""

    def __init__(self, config: FilterConfig):
        self.config = config
        self.roll = 0.0
        self.pitch = 0.0
        self.initialized = False

    @staticmethod
    def accel_angles(ax, ay, az):
        """Computing roll & pitch angles / Assumption: drone is at rest."""
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        return roll, pitch

    def compute_gyro_approx_angles(self, gx, gy, dt):
        """Approximation small angles : drone rotative velocities ~ Euler velocities."""
        roll_gyro = self.roll + gx * dt
        pitch_gyro = self.pitch + gy * dt
        return roll_gyro, pitch_gyro

    def compute_gyro_euler_angles(self, gx, gy, gz, dt):
        """Computing Euler angles derivative based on drone rotative velocities."""
        roll_rate = gx + gy * math.sin(self.roll)*math.tan(self.pitch) + gz * math.cos(self.roll)*math.tan(self.pitch)
        pitch_rate = gy * math.cos(self.roll) - gz * math.sin(self.roll)
        roll_gyro = self.roll + roll_rate * dt
        pitch_gyro = self.pitch + pitch_rate * dt
        return roll_gyro, pitch_gyro

    def update(self, gyro, accel, dt):
        """gyro = (gx, gy, gz) rad/s ; accel = (ax, ay, az) m/s² ; dt in s"""

        # Refusing update if time step unconsistent
        if dt <= 0.0 or dt > self.config.dt_max:
            return self.roll, self.pitch

        # Estimating real rotational speeds and accelerations by removing bias of sensors 
        gx, gy, gz = (g - b for g, b in zip(gyro, self.config.gyro_bias))
        ax, ay, az = (a - b for a, b in zip(accel, self.config.accel_bias))

        roll_acc, pitch_acc = self.accel_angles(ax, ay, az)

        # Initialization with accelerometer (drone assumed steady => measurements reliable)
        if not self.initialized:
            self.roll, self.pitch = roll_acc, pitch_acc
            self.initialized = True
            return self.roll, self.pitch

        # Prediction with gyro integration
        if self.config.integration_method == "approx_small_angles":
            roll_gyro, pitch_gyro = self.compute_gyro_approx_angles(gx, gy, dt)
        elif self.config.integration_method == "euler_angles":
            roll_gyro, pitch_gyro = self.compute_gyro_euler_angles(gx, gy, gz, dt)
        else:
            raise ValueError(f"Unknown gyro integration method: {self.config.integration_method}")
        

        # Accelerometer gating : checking if total acceleration close to g (rejecting influence of dynamics)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        trust_acc = abs(norm - G) / G < self.config.accel_gate

        if trust_acc:
            a = self.config.alpha
            self.roll = a * roll_gyro + (1 - a) * roll_acc
            self.pitch = a * pitch_gyro + (1 - a) * pitch_acc
        else:
            # In case of high dynamics, relying on gyro only
            self.roll, self.pitch = roll_gyro, pitch_gyro
        
        return self.roll, self.pitch



#-----------------| Test Bench |-----------------#

def simulate(duration=20.0, fs=200.0, seed=0, alpha=0.98, method="euler_angles",
             amp_roll=10, amp_pitch=10, yaw_rate=0.0, plot=True):
    """
    Sinusoidal motion + artificial noise and biais affecting gyro sensor.
    Compares the performance of complementary filter vs only relying on gyroscope.
    Plots the different outputs and errors associated to the two solutions.
    """
    
    rnd = random.Random(seed)
    dt = 1.0 / fs
    b_g = (0.02, -0.015, 0.01)   # rad/s (uncompensated/hided biais)
    b_a = (0.15, -0.10, 0.20)          # m/s² (uncompensated/hided biais)
    sig_gyro = 0.005                   # rad/s
    sig_acc = 0.25                     # m/s²

    config = FilterConfig(alpha=alpha)          # bias not indicated in filter 
    config.integration_method = method          # instance attribute: does not touch the class
    compfilter = ComplementaryFilter(config)

    # Initialisation
    err_roll_est, err_pitch_est, err_roll_gyro_only, err_pitch_gyro_only, roll_true_list, roll_est_list, roll_gyro_list = [], [], [], [], [], [], []
    t = 0.0
    t_list = []
    roll_gyro_only = 0
    pitch_gyro_only = 0

    while t < duration:

        t_list.append(t)

        # Calculating real roll & pitch and associated Euler angle rates
        roll =  np.radians(amp_roll) * math.sin(2 * math.pi * 0.3 * t)
        pitch = np.radians(amp_pitch) * math.sin(2 * math.pi * 0.2 * t + 1.0)
        w_roll = np.radians(amp_roll) * 2 * math.pi * 0.3 * math.cos(2 * math.pi * 0.3 * t)
        w_pitch = np.radians(amp_pitch) * 2 * math.pi * 0.2 * math.cos(2 * math.pi * 0.2 * t + 1.0)
        w_yaw = yaw_rate
        roll_true_list.append(roll)

        # True body rates (p, q, r) from Euler angle rates (ZYX convention)
        p = w_roll - w_yaw * math.sin(pitch)
        q = w_pitch * math.cos(roll) + w_yaw * math.sin(roll) * math.cos(pitch)
        r = -w_pitch * math.sin(roll) + w_yaw * math.cos(roll) * math.cos(pitch)

        # Simulating measurements by adding noise & bias to groundtruth
        gyro = (p + b_g[0] + rnd.gauss(0, sig_gyro),
                q + b_g[1] + rnd.gauss(0, sig_gyro),
                r + b_g[2] + rnd.gauss(0, sig_gyro))
        
        # Specific force measured at rest: projected in the local drone reference frame
        ax = -G * math.sin(pitch)
        ay = G * math.sin(roll) * math.cos(pitch)
        az = G * math.cos(roll) * math.cos(pitch)
        accel = (ax + b_a[0] + rnd.gauss(0, sig_acc),
                 ay + b_a[1] + rnd.gauss(0, sig_acc),
                 az + b_a[2] + rnd.gauss(0, sig_acc))

        roll_est, pitch_est = compfilter.update(gyro, accel, dt)
        roll_gyro_only += gyro[0] * dt
        pitch_gyro_only += gyro[1] * dt
        err_roll_est.append(roll_est - roll)
        err_pitch_est.append(pitch_est - pitch)
        err_roll_gyro_only.append(roll_gyro_only - roll)
        err_pitch_gyro_only.append(pitch_gyro_only - pitch)
        t += dt
        roll_est_list.append(roll_est)
        roll_gyro_list.append(roll_gyro_only)

    if plot:
        rms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))
        print(f"RMS error roll with only integrated gyro: {math.degrees(rms(err_roll_gyro_only)):6.2f}°")
        print(f"RMS error pitch with only integrated gyro: {math.degrees(rms(err_pitch_gyro_only)):6.2f}°")
        print(f"RMS error roll with complementary filter ({method}): {math.degrees(rms(err_roll_est)):6.2f}°")
        print(f"RMS error pitch with complementary filter ({method}): {math.degrees(rms(err_pitch_est)):6.2f}°")

        d = np.degrees
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        ax1.plot(t_list, d(roll_true_list), label="groundtruth")
        ax1.plot(t_list, d(roll_est_list), label="complementary filter")
        ax1.plot(t_list, d(roll_gyro_list), label="gyro alone")
        ax1.set_ylabel("roll (°)"); ax1.legend()
        ax2.plot(t_list, d(np.array(roll_est_list) - np.array(roll_true_list)), label="error filter (°)")
        ax2.plot(t_list, d(np.array(roll_gyro_list) - np.array(roll_true_list)), label="error gyro (°)")
        ax2.set_ylabel("filter vs gyro"); ax2.set_xlabel("t (s)"); ax2.legend()
        plt.show()

    return t_list, roll_true_list, roll_est_list, roll_gyro_list

def compare_methods(amp_angles=((8,8), (50,50)), alpha=0.98, yaw_rate=1.2):
    """
    Displays the results obtained with the complementary filter depending on which intergrating method for the gyro:
    Rigorous Euler angles VS Approximation of small angles
    Possibilty to plot multiple curves with different scale factors.
    """

    d = np.degrees
    fig, axes = plt.subplots(len(amp_angles), 1, sharex=True)
    for ax, amp in zip(axes, amp_angles):
        for method in ("approx_small_angles", "euler_angles"):
            t, roll_true, roll_est, _ = simulate(alpha=alpha, method=method, amp_roll=amp[0], amp_pitch=amp[1],
                                                 yaw_rate=yaw_rate, plot=False)
            ax.plot(t, d(roll_est), label=method)
        ax.plot(t, d(roll_true), "k--", label="groundtruth")
        ax.set_ylabel(f"roll (°), amp_roll={amp[0]}")
    axes[0].legend(); axes[-1].set_xlabel("t (s)")
    plt.show()



if __name__ == "__main__":
    simulate()
    compare_methods()