# Complementary Altitude Filter

Conventions: reference frame $X$ forward, $Y$ left, $Z$ up. Body frame $U$ forward, $V$ left, $W$ up. Altitude $z$ in m, velocity $v$ in m/s, accelerations in m/s².

Inputs: roll $\varphi$ and pitch $\theta$ (attitude filter), accelerometer $\mathbf{f}_b = (a_u, a_v, a_w)$, pressure $p$.

Outputs: relative altitude $\hat z$ and vertical velocity $\hat v$.

## 1. Barometer: pressure → relative altitude

Hydrostatic equilibrium and ideal gas ($\rho = pM/RT$):

$$\frac{dp}{dz} = -\rho g = -\frac{Mg}{RT}\,p \quad\Rightarrow\quad p(z) = p_{ref}\,e^{-z/H}, \qquad H = \frac{RT}{Mg} \approx 8.4\ \text{km}$$

($T \approx 288$ K, $M = 0.029$ kg/mol, constant $T$ assumed.) Inverting:

$$z = H \ln\frac{p_{ref}}{p} \approx H\,\frac{p_{ref} - p}{p_{ref}} \qquad (|z| \ll H)$$

- First-order error is about $z/(2H)$: 0.3 % at 50 m, negligible for a drone.
- Sensitivity: $|dz/dp| = H/p_{ref} \approx 0.083$ m/Pa, i.e. about 12 Pa per meter.
- $p_{ref}$ is the ground pressure of the day. It makes the altitude **relative to the take-off point** and absorbs the static offset.

Measurement model:

$$z_{baro} = z + b + n$$

- $n$: white noise, standard deviation $\sigma$ (measure it with the sensor at rest).
- $b$: slow drift (weather, temperature); negligible over short flights.
- Propeller wash can disturb $p$: handled by gating (section 6).

## 2. Accelerometer: vertical acceleration

The accelerometer measures the specific force $\mathbf{f} = \mathbf{a} - \mathbf{g}$ with $\mathbf{g} = (0,0,-G)$. In the world frame:

$$\mathbf{a} = R\,\mathbf{f}_b - \mathbf{g} \quad\Rightarrow\quad a_z = (R\,\mathbf{f}_b)_z - G$$

At rest: $\mathbf{f}_b = (0,0,G)$ so $a_z = 0$. ✔

Only the third row of $R = R_z(\psi)R_y(\theta)R_x(\varphi)$ is needed:

$$(-\sin\theta,\ \cos\theta\sin\varphi,\ \cos\theta\cos\varphi)$$

$$\boxed{a_z = -a_u\sin\theta + a_v\cos\theta\sin\varphi + a_w\cos\theta\cos\varphi - G}$$

- Yaw $\psi$ does not appear (rotation about the vertical leaves $a_z$ unchanged): no yaw estimate needed.
- Sensitivity to an attitude error $\delta$: the vertical error is second order, $\approx G\delta^2/2$ (1° gives about $1.5\cdot10^{-3}$ m/s²). With horizontal acceleration $a_h$ present it becomes $\approx a_h\,\delta$ (first order).
- An accelerometer bias $b_a$ adds directly to $a_z$.

## 3. Why one gain $\alpha$ is not enough

For attitude, the gyro gives the angle after **one** integration, so a first-order complementary filter is enough. Here the accelerometer gives the altitude after **two** integrations: $z_{acc} = a_z/s^2$.

Try $H(s) = \dfrac{1}{\tau s+1}$ (first order low-pass) on the barometer and $1-H = \dfrac{\tau s}{\tau s+1}$ (first order high-pass) on the inertial path:

$$\hat z = H\,z_{baro} + (1-H)\,\frac{a_z}{s^2} \quad\Rightarrow\quad \text{inertial path} = \frac{\tau}{s(\tau s+1)}\,a_z$$

A pure integrator remains at $s \to 0$: a constant bias $b_a$ gives a ramp, the altitude diverges.

Fix: the high-pass must have **two zeros at $s=0$** to cancel both integrations, so $H$ is second order.

## 4. Second-order complementary filter

Observer form (prediction by inertial integration, correction by the barometer on **both** states):

$$e = z_{baro} - \hat z, \qquad \dot{\hat z} = \hat v + k_1 e, \qquad \dot{\hat v} = a_z + k_2 e$$

Laplace transform, eliminating $\hat v$:

$$s\hat z = \hat v + k_1(z_{baro}-\hat z), \qquad s\hat v = a_z + k_2(z_{baro}-\hat z)$$

$$\hat z = \underbrace{\frac{k_1 s + k_2}{s^2 + k_1 s + k_2}}_{H(s)}\,z_{baro} + \underbrace{\frac{s^2}{s^2 + k_1 s + k_2}}_{1-H(s)}\,\frac{a_z}{s^2}$$

- $H + (1-H) = 1$: complementary. ✔ Barometer low-pass, accelerometer high-pass.
- $1-H \propto s^2$: both integrations are cancelled at low frequency, a bias no longer diverges.

**Choice of gains.** The common denominator $s^2 + k_1 s + k_2$ sets the dynamics. A double pole at $-\omega$ gives the fastest response without oscillation:

$$(s+\omega)^2 = s^2 + 2\omega s + \omega^2 \quad\Rightarrow\quad \boxed{k_1 = 2\omega,\quad k_2 = \omega^2}$$

One tuning parameter $\omega$ (rad/s), the crossover between "trust the barometer" and "trust the accelerometer". Response time $\approx 1/\omega$, crossover frequency $f = \omega/2\pi$.

## 5. Discrete implementation

At each step (time step $dt$):

1. Projection and clipping: $a_z \leftarrow \mathrm{clip}(a_z,\,\pm a_{z,max})$
2. Prediction: $\hat v \leftarrow \hat v + a_z\,dt$, then $\hat z \leftarrow \hat z + \hat v\,dt$
3. Innovation: $e = z_{baro} - \hat z$
4. Correction (if not rejected, see section 6):

$$\hat z \leftarrow \hat z + 2\omega\,dt\,e, \qquad \hat v \leftarrow \hat v + \omega^2\,dt\,e$$

Each continuous gain is multiplied by $dt$ (Euler). Validity: $\omega\,dt \ll 1$.

## 6. Gating

Gating acts on the **barometer innovation only**, never on the whole state. If the state were frozen on rejection, prediction and correction would both stop, $e$ would keep growing and the rejection would repeat forever (typically in free fall, where the state must keep following).

- Reject the correction if $|e| \ge$ `baro_gate` (propeller wash spike, outlier).
- Counter of consecutive rejections: after `max_baro_rejects` steps ($T = \text{max\_rejects}\cdot dt$), accept again: the inertial prediction has probably drifted, not the barometer.
- Free fall is consistent: $\mathbf{f}_b \approx 0$ so $a_z \approx -G$, the prediction follows the fall, and $a_{z,max}$ must be larger than $G$.
- `baro_gate`: a few $\sigma$ plus a margin for inertial drift during the rejection window.
- Time step gating: skip the update if $dt \le 0$ or $dt > dt_{max}$.

## 7. Choice of $\omega$

Trade-off between the two dominant errors.

**Bias error.** DC gain from $a_z$ to $\hat z$ is $1/k_2$: a constant accelerometer bias $b_a$ gives a static error

$$\Delta z = \frac{b_a}{\omega^2}$$

**Noise error.** White barometer noise (variance $\sigma^2$, period $dt$) goes through $H$:

$$\sigma_z^2 = \sigma^2\,dt\int_{-\infty}^{\infty}|H(j\Omega)|^2\frac{d\Omega}{2\pi}$$

With $\Omega = \omega x$: $|H|^2 = \dfrac{4x^2+1}{(x^2+1)^2}$ and $\displaystyle\int\frac{4x^2+1}{(x^2+1)^2}dx = 4\frac{\pi}{2}+\frac{\pi}{2} = \frac{5\pi}{2}$, hence

$$\sigma_z^2 = \frac{5}{4}\,\sigma^2\,dt\,\omega$$

**Minimization.**

$$J(\omega) = \frac{b_a^2}{\omega^4} + \frac{5}{4}\sigma^2\,dt\,\omega \quad\Rightarrow\quad \boxed{\omega^* = \left(\frac{16\,b_a^2}{5\,\sigma^2\,dt}\right)^{1/5}}$$

- Larger accelerometer bias gives larger $\omega^*$ (trust the barometer more). Noisier barometer or smaller $dt$ gives smaller $\omega^*$.
- Weak dependence (power $1/5$): the order of magnitude is enough.
- Compensating $b_a$ (calibration) moves the optimum toward smaller $\omega$ (smoother altitude).