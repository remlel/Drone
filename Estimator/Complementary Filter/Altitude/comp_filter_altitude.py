"""
Complementary altitude Filter: 
1) motor model
    Inputs: estimated roll & pitch, accelerometer output, barometer output
    Outputs: estimated altitude (and vertical velocity)
2)synthetic test bench.

Conventions: 
-> reference frame: X forward, Y left, Z up.
-> body frame: U forward, V left, W up.
Angles in radians, angular velocities in rad/s, accelerations in m/s², altitude in m.
All CONFIG parameters are arbitrary values to be calibrated experimentally with hardware.
"""

from dataclasses import dataclass
import math

G = 9.81



@dataclass
class FilterConfig:
    omega: float = 1.5                          # bandwidth of barometer correction rad/s (Tuning required)
    accel_bias: tuple = (0.0, 0.0, 0.0)         # m/s² (Tuning required)
    p_ref: float = 101325.0                     # reference pressure on ground Pa (Tuning required)
    dt_max: float = 0.05                        # gating of time steps s (Tuning required)
    baro_gate: float = 2.0                      # gating of barometer innovation m (Tuning required)
    max_baro_rejects: int = 25                  # consecutive rejections before accepting barometer again (Tuning required)
    az_max: float = 20.0                        # clipping of vertical acceleration m/s² (Tuning required)



class ComplementaryFilter:
    """Estimates altitude based on accelerometer & barometer."""
    
    def __init__(self, config: FilterConfig):
        self.config = config
        self.alt = 0.0              # Relative altitude (m)
        self.v = 0.0                # Vertical velocity (m/s)
        self.n_rejects = 0          # Consecutive barometer rejections

    def update(self, roll, pitch, accel, p, dt):
        """Estimated roll & pitch angles [rad]; accel = (ax, ay, az) [m/s²]; Pressure [Pa]; dt [s]"""

        # Refusing update if time step unconsistent
        if dt <= 0.0 or dt > self.config.dt_max:
            return self.alt

        # Estimating real accelerations by removing bias of accelerometer 
        au, av, aw = (a - b for a, b in zip(accel, self.config.accel_bias))

        # Computing motion vertical acceleration (without gravity)
        az = -au * math.sin(pitch) + av * math.sin(roll)*math.cos(pitch) + aw * math.cos(roll)*math.cos(pitch) - G

        # Clipping aberrant values (large enough to keep free fall)
        az = max(-self.config.az_max, min(self.config.az_max, az))

        # Prediction (inertial integration)
        self.v += az * dt
        self.alt += self.v * dt

        # Computing relative altitude based on barometer
        alt_baro = (8436*(self.config.p_ref - p))/self.config.p_ref

        # Innovation
        e = alt_baro - self.alt

        # Gating on innovation only: rejecting the barometer, never the whole state
        if abs(e) < self.config.baro_gate or self.n_rejects >= self.config.max_baro_rejects:
            # Correction (barometer)
            w = self.config.omega
            self.alt += 2 * w * dt * e
            self.v += w * w * dt * e
            self.n_rejects = 0
        else:
            self.n_rejects += 1
        
        return self.alt