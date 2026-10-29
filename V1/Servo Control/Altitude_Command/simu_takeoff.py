import numpy as np
import matplotlib.pyplot as plt

from loop_altitude import *


def test_dynamics_hover():
    """Sanity checks of VerticalDynamics: weight-compensating thrust gives no motion, in the air and on the ground."""

    config = ConfigDrone()

    in_air = VerticalDynamics(config, z_init=5.0)
    for _ in range(1000):
        in_air.resulting_motion(config.m_drone * G, 0.005)
    assert abs(in_air.z - 5.0) < 1e-9 and abs(in_air.v) < 1e-9

    on_ground = VerticalDynamics(config)
    for _ in range(1000):
        on_ground.resulting_motion(0.5 * config.m_drone * G, 0.005)     # Thrust below the weight
    assert on_ground.z == 0.0 and on_ground.v == 0.0


def run_takeoff(z_c: float = 2.0, climb_rate: float = 0.5, t_end: float = 12.0, dt: float = 0.005):
    """Closed loop with perfect measurements (z_est = z, v_est = v): isolates controller, motors and dynamics."""

    config = ConfigDrone()
    gains = ConfigController()

    reference = AltitudeReference(z_c, climb_rate)
    controller = AltitudeController(config, gains)
    motors = MotorModel(config)
    dynamics = VerticalDynamics(config)

    n = int(t_end / dt)
    log = np.zeros((n, 5))               # t, z_c, z, T_cmd, T_real

    for k in range(n):
        t = k * dt

        z_c_t = reference.step_command(t)
        # Later: sensors -> filters -> (z_est, v_est). For now, perfect measurements
        T_cmd = controller.controller_output(z_c_t, dynamics.z, dynamics.v, dt)
        T_real = motors.thrust_output(T_cmd, dt)
        dynamics.resulting_motion(T_real, dt)

        log[k] = [t, z_c_t, dynamics.z, T_cmd, T_real]

    return config, log


def plot_takeoff(config: ConfigDrone, log: np.ndarray):

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    ax1.plot(log[:, 0], log[:, 1], "--", label="z* (reference)")
    ax1.plot(log[:, 0], log[:, 2], label="z (real)")
    ax1.set_ylabel("Altitude [m]")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(log[:, 0], log[:, 3], "--", label="T_cmd")
    ax2.plot(log[:, 0], log[:, 4], label="T_real")
    ax2.axhline(config.m_drone * G, color="k", linewidth=0.8, label="weight m*g")
    ax2.axhline(config.T_max, color="r", linewidth=0.8, label="T_max")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Thrust [N]")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_dynamics_hover()
    config, log = run_takeoff()
    plot_takeoff(config, log)