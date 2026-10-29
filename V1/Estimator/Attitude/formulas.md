# Complementary Attitude Filter (roll / pitch)

Attitude estimation from a gyroscope and an accelerometer. Yaw is not estimated.

## 1. Conventions

- Body frame: $X$ forward, $Y$ left, $Z$ up. Euler angles ZYX: yaw $\psi$, pitch $\theta$, roll $\phi$.
- Gyro measures body rates $(p, q, r)$ in rad/s. Accelerometer measures specific force $\mathbf{f}=(a_x, a_y, a_z)$ in m/s².
- $g = 9.81\ \text{m/s}^2$, sample period $dt$, discrete step $k$.

## 2. Sensor models

$$\boldsymbol{\omega}_m = \boldsymbol{\omega} + \mathbf{b}_g + \mathbf{n}_g, \qquad \mathbf{n}_g \sim \mathcal{N}(0, \sigma_g^2)$$

$$\mathbf{f}_m = \mathbf{f} + \mathbf{b}_a + \mathbf{n}_a, \qquad \mathbf{n}_a \sim \mathcal{N}(0, \sigma_a^2)$$

At rest (no linear acceleration), the accelerometer measures the gravity reaction projected in the body frame:

$$\mathbf{f} = g\begin{pmatrix} -\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta \end{pmatrix}$$

Biases $\mathbf{b}_g$, $\mathbf{b}_a$ are slowly varying (treated as constant here). Calibrated measurements: $\boldsymbol{\omega}_c = \boldsymbol{\omega}_m - \hat{\mathbf{b}}_g$, $\mathbf{f}_c = \mathbf{f}_m - \hat{\mathbf{b}}_a$.

## 3. Attitude from the accelerometer

$$\phi_{acc} = \operatorname{atan2}(a_y, a_z), \qquad \theta_{acc} = \operatorname{atan2}\!\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$

- Valid only if the sensor measures gravity alone: $\big|\,\|\mathbf{f}\| - g\,\big| / g < \varepsilon$ (gating, otherwise the measurement is ignored).
- No drift, but noisy and corrupted by linear accelerations and vibrations.
- Yaw is unobservable (gravity is invariant to rotation about the vertical).

## 4. Attitude propagation from the gyro

**Exact (Euler kinematics, ZYX):**

$$\dot\phi = p + (q\sin\phi + r\cos\phi)\tan\theta, \qquad \dot\theta = q\cos\phi - r\sin\phi$$

**Small-angle approximation:**

$$\dot\phi \approx p, \qquad \dot\theta \approx q$$

Error of the approximation, with $\dot\psi$ the yaw rate:

- roll rate error $= (q\sin\phi + r\cos\phi)\tan\theta = \dot\psi\sin\theta$
- if $\dot\psi = 0$: roll rate is exact, pitch rate error $= -\dot\theta(1-\cos\phi)$

Hence the approximation degrades with large angles, and mostly when the yaw rate is non-zero.

Discrete integration (forward Euler): $\phi_k^- = \hat\phi_{k-1} + \dot\phi\,dt$ (same for $\theta$).
Singularity at $\theta \to \pm 90^\circ$ ($\tan\theta$ diverges): quaternions avoid it.

## 5. Complementary filter

$$\boxed{\hat\theta_k = \alpha\,\big(\hat\theta_{k-1} + \dot\theta\,dt\big) + (1-\alpha)\,\theta_{acc,k}}$$

(same for $\phi$), with $\alpha \in [0,1)$.

**Predict / correct form** (a proportional correction toward the accelerometer, gain $1-\alpha$):

$$\hat\theta_k = \theta_k^- + (1-\alpha)\,\big(\theta_{acc,k} - \theta_k^-\big), \qquad \theta_k^- = \hat\theta_{k-1} + \dot\theta\,dt$$

**Time constant and cutoff frequency:**

$$\tau = \frac{\alpha\,dt}{1-\alpha}, \qquad \alpha = \frac{\tau}{\tau + dt}, \qquad f_c = \frac{1}{2\pi\tau}$$

| $\alpha$ ($dt = 5$ ms) | 0.9 | 0.95 | 0.98 | 0.99 | 0.999 |
|---|---|---|---|---|---|
| $\tau$ (s) | 0.045 | 0.095 | 0.245 | 0.495 | 4.995 |
| $f_c$ (Hz) | 3.5 | 1.7 | 0.65 | 0.32 | 0.03 |

**Frequency view** (continuous time, $\tau\dot{\hat\theta} + \hat\theta = \tau\dot\theta_m + \theta_{acc}$):

$$\hat\theta(s) = \underbrace{\frac{\tau s}{\tau s + 1}}_{\text{high-pass}}\cdot\frac{\dot\theta_m(s)}{s} \;+\; \underbrace{\frac{1}{\tau s + 1}}_{\text{low-pass}}\,\theta_{acc}(s)$$

The gyro is trusted above $f_c$ (short term), the accelerometer below $f_c$ (long term). The two filters sum to 1: with perfect sensors the output equals the true angle for any $\alpha$. $\alpha$ only matters because sensors are imperfect.

## 6. Effect of sensor errors

### 6.1 Gyro bias: bounded offset

Assume a perfect accelerometer and a gyro bias $b$. With $e_k = \hat\theta_k - \theta_k$:

$$e_k = \alpha\,(e_{k-1} + b\,dt) \;\Longrightarrow\; e_k = b\tau\,(1-\alpha^k) \;\xrightarrow{k\to\infty}\; \boxed{e_\infty = \frac{\alpha\,b\,dt}{1-\alpha} = b\tau}$$

Interpretation: each step the gyro injects an error $b\,dt$, the correction removes a fraction $(1-\alpha)$ of the existing error. The error settles where injection equals removal. Convergence takes a few $\tau$.

- The bias stays **constant in time**, but its value is **proportional to $\tau$**: larger $\alpha$ means larger offset.

### 6.2 Accelerometer bias: independent of $\alpha$

The accelerometer path has a DC gain of 1, so the filter fully inherits its static error:

$$e_\infty^{\phi} \approx \frac{b_{a,y}}{g}, \qquad e_\infty^{\theta} \approx -\frac{b_{a,x}}{g} \qquad (\text{small angles})$$

Example: $0.1$ m/s² gives $\approx 0.58^\circ$. Only calibration (e.g. 6 positions) reduces it, not $\alpha$.

### 6.3 Noise

- Accelerometer white noise (angle noise $\sigma_\theta \approx \sigma_a/g$) is low-passed:

$$\sigma_{out} = \sigma_\theta\sqrt{\frac{1-\alpha}{1+\alpha}} \;\approx\; \sigma_\theta\sqrt{\frac{dt}{2\tau}}$$

  Kept fraction: 58 % for $\alpha = 0.5$, 23 % for 0.9, 10 % for 0.98.
- Gyro noise is high-passed through the filter and stays bounded (see section 8 for the gyro alone).

### 6.4 Linear accelerations and vibrations

They corrupt $\theta_{acc}$: handled by gating ($\|\mathbf{f}\| \approx g$), by a larger $\tau$, and by mechanical damping of the IMU.

## 7. Choice of $\alpha$

Trade-off between the two dominant errors:

- larger $\alpha$ (larger $\tau$): less accelerometer noise, but larger gyro-bias offset $b\tau$;
- smaller $\alpha$: bias corrected fast, but more noise and more sensitivity to vibrations.

Minimizing $J(\tau) = (b\tau)^2 + \sigma_\theta^2\,\dfrac{dt}{2\tau}$:

$$\tau^* = \left(\frac{\sigma_\theta^2\,dt}{4\,b^2}\right)^{1/3}$$

- Larger gyro bias gives smaller $\tau^*$. Noisier accelerometer or better gyro gives larger $\tau^*$.
- Compensating $\mathbf{b}_g$ (calibration) moves the optimum toward $\alpha \to 1$.

## 8. Accelerometer gating

The accelerometer only measures gravity when the vehicle is not accelerating. Under linear acceleration (or free fall), $\|\mathbf{a}\|$ departs from $g$ and the accelerometer-based angles become unreliable. Let

$$\rho = \frac{\|\mathbf{a}\|}{g}$$

and let `accel_gate` $= 1 + \varepsilon$ be the tolerated deviation. Instead of a binary accept/reject rule, the gyro weight is blended continuously:

$$a = \alpha + (1-\alpha)\,\mathrm{clip}\!\left(\frac{|\rho - 1|}{\varepsilon},\,0,\,1\right)$$

$$\theta = a\,\theta_{\text{gyro}} + (1-a)\,\theta_{\text{acc}}$$

- $\rho = 1$: nominal $\alpha$ (see section 7).
- $|\rho - 1| \ge \varepsilon$: pure gyro integration ($a = 1$), so the estimate is continuous at the gate (no jumps).
- The test is symmetric around $\rho = 1$: free fall ($\rho < 1$) is rejected like strong acceleration ($\rho > 1$).

Tuning notes:

- $\varepsilon$ must be larger than the vibration-induced fluctuation of $\rho$ in hover, otherwise the accelerometer is rejected permanently. Measure the standard deviation of $\rho$ on the ground (motors on) and in hover, and set $\varepsilon$ to a few standard deviations.
- If vibrations are significant, low-pass $\|\mathbf{a}\|$ before computing $\rho$.

## 9. Sidebar: why not the gyro alone

Pure integration of the gyro:

$$\theta_{gyro}(t) = \theta_0 + \int_0^t (\omega + b + n_g)\,d\tau' = \theta(t) + \underbrace{b\,t}_{\text{linear drift}} + \underbrace{\sigma_g\sqrt{t\,dt}}_{\text{random walk (std)}}$$

- The error is **unbounded**: it grows linearly with the bias and as $\sqrt{t}$ with the noise.

| | Gyro | Accelerometer |
|---|---|---|
| Short term | accurate, smooth | noisy, sensitive to vibrations |
| Long term | drifts | no drift (gravity reference) |
| Perturbed by linear acceleration | no | yes |

The filter takes the best of each: high frequencies from the gyro, low frequencies from the accelerometer.

## 10. Algorithm (one step)

1. Reject the step if $dt \le 0$ or $dt > dt_{max}$.
2. Calibrate: $\boldsymbol{\omega}_c = \boldsymbol{\omega}_m - \hat{\mathbf{b}}_g$, $\mathbf{f}_c = \mathbf{f}_m - \hat{\mathbf{b}}_a$.
3. Compute $\phi_{acc}, \theta_{acc}$ (section 3). At the first step, initialize $\hat\phi, \hat\theta$ with them (drone at rest).
4. Predict $\phi^-, \theta^-$ with the chosen integration method (section 4).
5. If $|\,\|\mathbf{f}_c\| - g\,|/g < \varepsilon$: fuse with $\alpha$ (section 5); otherwise keep the prediction.

