from comp_filter_altitude import *
import numpy as np
import matplotlib.pyplot as plt

#-----------------| Test Bench |-----------------#

P_REF = 101325.0        # reference pressure on ground (Pa)
H = 8436.0              # scale height of the atmosphere (m)

def rms(x):
    """Root mean square of an array"""
    return float(np.sqrt(np.mean(np.square(x))))



# ---| Displays |--- #

def plot_run(res, title="Altitude estimation"):
    """Ground truth vs estimate vs raw barometer, plus velocity and altitude error.
    res: dict of arrays (t, z_true, v_true, z_hat, v_hat, z_baro)"""
    t = res["t"]
    err_z = res["z_hat"] - res["z_true"]
    err_v = res["v_hat"] - res["v_true"]

    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle(title)

    # Altitude: ground truth vs estimate vs raw barometer
    axs[0].plot(t, res["z_baro"], color="lightgray", label="barometer (raw)")
    axs[0].plot(t, res["z_true"], "k--", label="ground truth")
    axs[0].plot(t, res["z_hat"], "r", label="estimate")
    axs[0].set_ylabel("altitude (m)")
    axs[0].legend()
    axs[0].grid(True)

    # Vertical velocity: ground truth vs estimate
    axs[1].plot(t, res["v_true"], "k--", label="ground truth")
    axs[1].plot(t, res["v_hat"], "r", label=f"estimate (RMS error = {rms(err_v):.3f} m/s)")
    axs[1].set_ylabel("velocity (m/s)")
    axs[1].legend()
    axs[1].grid(True)

    # Altitude error
    axs[2].plot(t, err_z, "r", label=f"RMS = {rms(err_z):.3f} m, max = {np.max(np.abs(err_z)):.3f} m")
    axs[2].axhline(0.0, color="k", linewidth=0.8)
    axs[2].set_ylabel("altitude error (m)")
    axs[2].set_xlabel("time (s)")
    axs[2].legend()
    axs[2].grid(True)

    fig.tight_layout()
    return fig


def plot_comparison(res, z_baro_only, z_inertial_only, title="Estimators comparison"):
    """Fusion vs barometer only vs inertial only (drifting with the uncorrected bias)"""
    t = res["t"]

    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title)

    # Altitude
    axs[0].plot(t, res["z_true"], "k--", label="ground truth")
    axs[0].plot(t, z_baro_only, color="lightgray", label="barometer only")
    axs[0].plot(t, z_inertial_only, "b", label="inertial only")
    axs[0].plot(t, res["z_hat"], "r", label="complementary filter")
    axs[0].set_ylabel("altitude (m)")
    axs[0].legend()
    axs[0].grid(True)

    # Absolute errors
    axs[1].plot(t, np.abs(z_baro_only - res["z_true"]), color="lightgray",
                label=f"barometer only (RMS = {rms(z_baro_only - res['z_true']):.3f} m)")
    axs[1].plot(t, np.abs(z_inertial_only - res["z_true"]), "b",
                label=f"inertial only (RMS = {rms(z_inertial_only - res['z_true']):.3f} m)")
    axs[1].plot(t, np.abs(res["z_hat"] - res["z_true"]), "r",
                label=f"complementary filter (RMS = {rms(res['z_hat'] - res['z_true']):.3f} m)")
    axs[1].set_ylabel("|altitude error| (m)")
    axs[1].set_xlabel("time (s)")
    axs[1].legend()
    axs[1].grid(True)

    fig.tight_layout()
    return fig


def plot_omega_sweep(omegas, rms_z, omega_star=None):
    """RMS altitude error vs omega: U-shaped curve (bias on the left, noise on the right)"""
    omegas = np.asarray(omegas)
    rms_z = np.asarray(rms_z)
    i_min = int(np.argmin(rms_z))

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.semilogx(omegas, rms_z, "o-", label="simulation")
    ax.plot(omegas[i_min], rms_z[i_min], "r*", markersize=14,
            label=f"measured optimum: omega = {omegas[i_min]:.2f} rad/s")
    if omega_star is not None:
        ax.axvline(omega_star, color="k", linestyle="--",
                   label=f"theoretical omega* = {omega_star:.2f} rad/s")
    ax.set_xlabel("omega (rad/s)")
    ax.set_ylabel("altitude RMS error (m)")
    ax.set_title("Choice of omega")
    ax.legend()
    ax.grid(True, which="both")

    fig.tight_layout()
    return fig


def plot_bias_check(biases, err_measured, omega):
    """Steady-state altitude error vs accelerometer bias: theory predicts b / omega^2"""
    biases = np.asarray(biases)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(biases, biases / omega**2, "k--", label="theory: b / omega^2")
    ax.plot(biases, err_measured, "ro", label="simulation")
    ax.set_xlabel("accelerometer bias (m/s²)")
    ax.set_ylabel("steady-state altitude error (m)")
    ax.set_title(f"Static error due to uncorrected bias (omega = {omega} rad/s)")
    ax.legend()
    ax.grid(True)

    fig.tight_layout()
    return fig



# ---| Simulation |--- #

def trajectory(t, scenario):
    """Ground truth altitude, velocity and vertical acceleration (analytic, no numerical derivative).
    The motion is purely vertical in the world frame."""
    if scenario == "freefall":
        # Hovering until t0, then free fall (a = -G)
        t0 = 1.0
        tau = np.maximum(t - t0, 0.0)
        return -0.5 * G * tau**2, -G * tau, -G * (t >= t0).astype(float)

    if scenario in ("rest", "level"):
        zeros = np.zeros_like(t)
        return zeros, zeros, zeros

    # Smooth: two slow sinusoidal motions starting with z = 0 and v = 0
    A, W1, B, W2 = 5.0, 0.3, 1.0, 1.2
    z = A * (1 - np.cos(W1 * t)) + B * (1 - np.cos(W2 * t))
    v = A * W1 * np.sin(W1 * t) + B * W2 * np.sin(W2 * t)
    a = A * W1**2 * np.cos(W1 * t) + B * W2**2 * np.cos(W2 * t)
    return z, v, a


def attitude(t, scenario):
    """True roll and pitch (rad)"""
    if scenario == "rest":
        # Tilted and motionless: tests the signs of the projection
        return 0.2 * np.ones_like(t), -0.1 * np.ones_like(t)
    if scenario == "level":
        return np.zeros_like(t), np.zeros_like(t)
    return np.deg2rad(10) * np.sin(0.5 * t), np.deg2rad(5) * np.sin(0.3 * t + 1.0)


def rotation(roll, pitch):
    """Body -> world rotation matrix R = Ry(pitch) @ Rx(roll), yaw = 0.
    Built explicitly (not reusing the projection line of the filter) to catch sign errors"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx


def simulate(scenario="smooth", T=40.0, dt=0.01, seed=0, sigma_baro=0.3, sigma_acc=0.05,
             accel_bias=(0.0, 0.0, 0.0), baro_bias=0.0):
    """Synthetic sensors: accelerometer (body frame, specific force) and barometer (pressure).
    sigma_baro in m, sigma_acc in m/s², accel_bias in m/s² (body frame), baro_bias in m"""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, T, dt)
    z, v, a = trajectory(t, scenario)
    roll, pitch = attitude(t, scenario)

    # Accelerometer: f_body = R^T (a + G e_z) + bias + noise  (at rest: f_body = (0, 0, G) when level)
    accel = np.zeros((len(t), 3))
    for k in range(len(t)):
        f_world = np.array([0.0, 0.0, a[k] + G])
        accel[k] = rotation(roll[k], pitch[k]).T @ f_world
    accel += np.array(accel_bias) + rng.normal(0.0, sigma_acc, accel.shape)

    # Barometer: z_baro = z + b + n, converted to pressure with the exact exponential law
    z_meas = z + baro_bias + rng.normal(0.0, sigma_baro, len(t))
    p = P_REF * np.exp(-z_meas / H)

    return dict(t=t, dt=dt, z_true=z, v_true=v, roll=roll, pitch=pitch, accel=accel, p=p)


def run_filter(sim, cfg, p=None):
    """Runs the altitude filter on simulated data (optionally on a modified pressure signal)"""
    if p is None:
        p = sim["p"]
    filt = ComplementaryFilter(cfg)
    n = len(sim["t"])
    z_hat = np.zeros(n)
    v_hat = np.zeros(n)
    for k in range(n):
        z_hat[k] = filt.update(sim["roll"][k], sim["pitch"][k], sim["accel"][k], p[k], sim["dt"])
        v_hat[k] = filt.v

    # Raw barometer altitude (same first-order conversion as in the filter)
    z_baro = H * (P_REF - p) / P_REF
    return dict(t=sim["t"], z_true=sim["z_true"], v_true=sim["v_true"],
                z_hat=z_hat, v_hat=v_hat, z_baro=z_baro)



# ---| Main |--- #

def main():
    omega = 1.5
    cfg = FilterConfig(omega=omega, p_ref=P_REF)
    bias_acc = (0.02, -0.03, 0.1)       # uncorrected accelerometer bias (m/s²)

    # 1) Rest test: tilted, motionless, no noise -> altitude and velocity must stay at 0
    sim = simulate("rest", T=5.0, sigma_baro=0.0, sigma_acc=0.0)
    res = run_filter(sim, cfg)
    max_z = np.max(np.abs(res["z_hat"]))
    max_v = np.max(np.abs(res["v_hat"]))
    print(f"[1] Rest test: max |z| = {max_z:.2e} m, max |v| = {max_v:.2e} m/s -> "
          f"{'OK' if max_z < 1e-9 and max_v < 1e-9 else 'FAILED (check signs / gravity)'}")

    # 2) Ground truth vs filter, with noise and an uncorrected accelerometer bias
    sim = simulate("smooth", seed=1, accel_bias=bias_acc)
    res = run_filter(sim, cfg)
    plot_run(res, title=f"Altitude estimation (omega = {omega} rad/s, uncorrected accel bias)")

    # 3) Fusion vs barometer only vs inertial only (omega = 0: no correction, pure integration)
    res_inertial = run_filter(sim, FilterConfig(omega=0.0, p_ref=P_REF))
    plot_comparison(res, res["z_baro"], res_inertial["z_hat"])

    # 4) Static error due to an uncorrected accelerometer bias: theory = b / omega^2
    biases = np.linspace(-0.2, 0.2, 9)
    errs = []
    for b in biases:
        s = simulate("level", T=20.0, sigma_baro=0.0, sigma_acc=0.0, accel_bias=(0.0, 0.0, b))
        r = run_filter(s, cfg)
        last = r["t"] > 15.0                                 # steady state
        errs.append(np.mean(r["z_hat"][last] - r["z_true"][last]))
    plot_bias_check(biases, errs, omega)

    # 5) Sweep of omega: RMS error averaged over several noise seeds
    b_sweep, sigma_baro, dt = 0.1, 0.3, 0.01
    sims = [simulate("smooth", seed=s, sigma_baro=sigma_baro, accel_bias=(0.0, 0.0, b_sweep))
            for s in range(5)]
    omegas = np.logspace(-1, 1, 25)
    rms_z = []
    for w in omegas:
        # Barometer gating disabled here to isolate the theoretical trade-off
        cfg_w = FilterConfig(omega=w, p_ref=P_REF, baro_gate=1e9)
        errs_w = [rms(run_filter(s, cfg_w)["z_hat"] - s["z_true"]) for s in sims]
        rms_z.append(np.mean(errs_w))
    omega_star = (16 * b_sweep**2 / (5 * sigma_baro**2 * dt)) ** 0.2
    plot_omega_sweep(omegas, rms_z, omega_star)
    print(f"[5] omega: measured optimum = {omegas[int(np.argmin(rms_z))]:.2f} rad/s, "
          f"theoretical omega* = {omega_star:.2f} rad/s")

    # 6) Gating: isolated pressure spikes (+8 m) and a permanent barometer jump (+5 m) at t = 35 s
    sim = simulate("smooth", seed=2, accel_bias=bias_acc)
    p = sim["p"].copy()
    for idx in (800, 1500, 2200, 3000):
        p[idx] *= np.exp(-8.0 / H)
    p[3500:] *= np.exp(-5.0 / H)
    res_gate = run_filter(sim, cfg, p=p)
    plot_run(res_gate, title="Gating: spikes rejected, barometer jump accepted after max_baro_rejects")

    # 7) Free fall: the estimate must keep following (no lock of the filter)
    sim = simulate("freefall", T=3.0, seed=3, accel_bias=bias_acc)
    res_ff = run_filter(sim, cfg)
    print(f"[7] Free fall: final error = {res_ff['z_hat'][-1] - res_ff['z_true'][-1]:.3f} m "
          f"(true altitude = {res_ff['z_true'][-1]:.2f} m)")
    plot_run(res_ff, title="Free fall")

    plt.show()


if __name__ == "__main__":
    main()