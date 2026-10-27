from dataclasses import dataclass
import numpy as np

G = 9.81                          # Gravity [m/s^2]



@dataclass
class ConfigDrone:

    # Drone global
    m_drone: float = 1.0          # Mass of drone [kg]

    # Drone motors
    T_max: float = 20.0           # Maximum total thrust [N] (thrust-to-weight ratio of about 2)
    tau_motor: float = 0.05       # Motor time constant (first-order lag) [s]

    # Drone sensors
    sigma_gyro: float = 0.005     # Gyroscope noise std [rad/s]
    sigma_acce: float = 0.05      # Accelerometer noise std [m/s^2]
    sigma_baro: float = 3.0       # Barometer noise std [Pa] (about 0.25 m of altitude)
    p_ref: float = 101325.0       # Pressure at z = 0 [Pa]
    seed: int = 0                 # Seed of the noise generator (reproducible runs)



@dataclass
class ConfigController:

    # Gains of the altitude PID, whose output is an acceleration [m/s^2]
    Kp: float = 4.0               # Proportional gain [1/s^2]: wn^2 with wn = 2 rad/s
    Kd: float = 2.8               # Derivative gain [1/s]: 2*zeta*wn with zeta = 0.7
    Ki: float = 1.0               # Integral gain [1/s^3]



class AltitudeReference:
    """Altitude command: ramp from 0 to z_c at a limited climb rate (a step would demand an unrealistic thrust)."""

    def __init__(self, z_c: float, climb_rate: float):
        self.z_c = z_c                  # Targeted altitude [m]
        self.climb_rate = climb_rate    # Maximum climb rate [m/s]

    def step_command(self, t: float):
        """Returns altitude command [m] for a given time stamp [s]."""
        return min(self.climb_rate * t, self.z_c)



class AltitudeController:
    """Contains error summer, PID regulator (output = desired acceleration), associated thrust command and saturator."""

    def __init__(self, config: ConfigDrone, gains: ConfigController):

        self.config = config
        self.gains = gains
        self.sum_error = 0.0     # Integral of the error [m.s], for the integral correction

    def controller_output(self, z_c: float, z_est: float, v_est: float, dt: float):
        """Returns the saturated thrust command [N] from the altitude command and the estimates (z, v)."""

        error = z_c - z_est                                                   # Altitude error [m]

        # Desired acceleration [m/s^2]; derivative term acts on the estimated speed (no noisy derivative of the error)
        a = self.gains.Kp * error + self.gains.Ki * self.sum_error - self.gains.Kd * v_est

        T_raw = self.config.m_drone * (a + G)                                 # Thrust that would give this acceleration [N]
        T = min(max(T_raw, 0.0), self.config.T_max)                           # Saturation: motors can only push, up to T_max

        # Anti-windup: integrate only if not saturated, or if the error pulls the output back out of saturation
        if T == T_raw or (T_raw > T and error < 0.0) or (T_raw < T and error > 0.0):
            self.sum_error += error * dt

        return T



class MotorModel:
    """Thrust response to a thrust command: saturation and first-order lag (time constant tau_motor)."""

    def __init__(self, config: ConfigDrone):
        self.config = config
        self.T_real = 0.0        # Real total thrust [N] (motors are off at start)

    def thrust_output(self, T_cmd: float, dt: float):
        """Returns the real total thrust [N] after dt seconds."""

        T_cmd = min(max(T_cmd, 0.0), self.config.T_max)              # Motors cannot exceed their limits
        alpha = 1.0 - np.exp(-dt / self.config.tau_motor)            # Exact discretization of the lag (= dt/tau for small dt)
        self.T_real += alpha * (T_cmd - self.T_real)

        return self.T_real



class VerticalDynamics:
    """Vertical motion of the drone (level attitude): m * a = T - m * g, with the ground as a floor."""

    def __init__(self, config: ConfigDrone, z_init: float = 0.0, v_init: float = 0.0):
        self.config = config
        self.z = z_init          # Altitude of the drone [m]
        self.v = v_init          # Vertical speed of the drone [m/s]
        self.a = 0.0             # Vertical acceleration [m/s^2], kept for the accelerometer model

    def resulting_motion(self, T_real: float, dt: float):
        """Returns the real altitude [m] and vertical speed [m/s] after dt seconds."""

        a = T_real / self.config.m_drone - G                 # Acceleration from Newton's law along Z [m/s^2]

        if self.z <= 0.0 and a < 0.0:                        # On the ground and thrust below the weight: the ground carries the rest
            self.z, self.v, a = 0.0, 0.0, 0.0
        else:
            self.z += self.v * dt + a * dt**2 / 2            # Exact for a constant acceleration over dt
            self.v += a * dt
            if self.z < 0.0:                                 # Touching down during this step
                self.z, self.v, a = 0.0, 0.0, 0.0

        self.a = a

        return self.z, self.v



class Sensors:
    """Simulated sensors: they turn the real state into noisy measurements (simulation only, not ported to C++)."""

    def __init__(self, config: ConfigDrone):
        self.config = config
        self.rng = np.random.default_rng(config.seed)

    def gyroscope(self):
        """Not needed while the attitude is held level (true angular rates are zero): added with the attitude step."""
        raise NotImplementedError

    def accelerometer(self, a_real: float):
        """Returns the measured acceleration along the body vertical axis [m/s^2].

        An accelerometer measures the specific force (thrust / mass), not the acceleration: it reads +g at rest,
        so measurement = a_real + g + noise (Z up, level attitude).
        """
        return a_real + G + self.rng.normal(0.0, self.config.sigma_acce)

    def barometer(self, z_real: float):
        """Returns the measured pressure [Pa] for a given real altitude [m]."""

        p_real = self.config.p_ref * (1 - z_real / 8436)                       # First-order expansion of p0 * exp(-z/H), H ~ 8.4 km
        p_meas = p_real + self.rng.normal(0.0, self.config.sigma_baro)         # Real pressure + noise

        return p_meas