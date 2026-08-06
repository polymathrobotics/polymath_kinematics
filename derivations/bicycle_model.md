# Bicycle Model (Ackermann) Kinematics

A front-steered, rear-driven vehicle modeled as if both front wheels collapse to a single virtual wheel on the centerline (the "bicycle" simplification). The full four-wheel speed computation then re-expands using Ackermann geometry relative to the instantaneous center of rotation.

## Parameters

| Symbol | Code name | Description |
|---|---|---|
| $L$ | `wheelbase_m` | Front axle to rear axle distance |
| $W$ | `track_width_m` | Distance between the centers of the left and right wheels |
| $r$ | `wheel_radius_m` | Wheel radius |

## Coordinate convention

- $v$ — forward velocity at the rear axle center (positive = forward)
- $\omega$ — yaw rate of the body (positive = counter-clockwise / left turn)
- $\delta$ — front steering angle (positive = left turn)
- $R$ — turning radius measured from the ICR to the rear axle center (positive = left turn, negative = right turn)

## Forward kinematics (steering → body velocity)

The bicycle model relates the steering angle to the yaw rate through the wheelbase:

$$\omega = \frac{v \cdot \tan(\delta)}{L}$$

This follows from the geometry: the rear axle center traces a circle of radius $R = L / \tan(\delta)$, and the yaw rate is $\omega = v / R$.

## Inverse kinematics (body velocity → steering state)

### Steering angle

Invert the forward relation:

$$\delta = \arctan\!\left(\frac{\omega \cdot L}{v}\right)$$

$\arctan$ (not $\operatorname{atan2}$) is used intentionally: the result must lie in $(-\pi/2,\, \pi/2)$, which is the physically valid range for a steering angle. $\operatorname{atan2}(\omega L, v)$ would add $\pm\pi$ when $v < 0$, placing $\delta$ in the wrong quadrant during reverse motion.

### Turning radius

$$R = \frac{L}{\tan(\delta)}$$

When $\delta \to 0$, $R \to \infty$ (straight line). The sign of $R$ matches the sign of $\delta$.

### Inverse conversion

$$\delta = \arctan\!\left(\frac{L}{R}\right)$$

## Four-wheel speeds via ICR geometry

The ICR lies on the line extending the rear axle, at signed distance $R$ from the rear axle center. Every point on the rigid body has ground speed equal to $\omega$ times its distance from the ICR.

### Rear wheels

The rear wheels sit on the axle at $\pm\, W/2$ from center, directly along the ICR line:

$$d_{\text{rl}} = R - \frac{W}{2} \qquad d_{\text{rr}} = R + \frac{W}{2}$$

### Front wheels

The front wheels are offset longitudinally by $L$ from the rear axle. Their distance from the ICR is the hypotenuse of the lateral offset and the wheelbase:

$$d_{\text{fl}} = \operatorname{copysign}\!\left(\sqrt{\left(R - \tfrac{W}{2}\right)^2 + L^2},\; R - \tfrac{W}{2}\right)$$

$$d_{\text{fr}} = \operatorname{copysign}\!\left(\sqrt{\left(R + \tfrac{W}{2}\right)^2 + L^2},\; R + \tfrac{W}{2}\right)$$

$\operatorname{copysign}$ preserves the sign of the lateral offset. This matters when the ICR falls between the rear wheels ($|R| < W/2$) — in that case the inner wheel must spin in the opposite direction.

### Wheel angular velocities

Convert each wheel's ground speed to angular velocity:

$$\omega_{\text{wheel}} = \frac{\omega \cdot d_{\text{ICR}}}{r}$$

Applied to all four wheels:

$$\omega_{\text{rl}} = \frac{\omega \left(R - W/2\right)}{r}$$

$$\omega_{\text{rr}} = \frac{\omega \left(R + W/2\right)}{r}$$

$$\omega_{\text{fl}} = \frac{\omega \cdot \operatorname{copysign}\!\left(\sqrt{(R - W/2)^2 + L^2},\; R - W/2\right)}{r}$$

$$\omega_{\text{fr}} = \frac{\omega \cdot \operatorname{copysign}\!\left(\sqrt{(R + W/2)^2 + L^2},\; R + W/2\right)}{r}$$

## Edge cases in the implementation

| Condition | Behavior |
|---|---|
| $v \approx 0$ | Bicycle model is degenerate — cannot determine $\delta$. Returns $\delta = 0$, $R = \infty$, all wheel speeds $= 0$ |
| $\omega \approx 0,\; v \neq 0$ | Straight line. All wheels spin at $v / r$ |
| $\|R\| < W/2$ | ICR between rear wheels. Inner wheel reverses direction (handled by $\operatorname{copysign}$) |

## Forward projection (`BicycleProjector`)

`BicycleProjector` wraps this model to roll a pose forward in time under a rate-limited steering
actuator.

**Pose reference.** The pose is the **rear axle** centre, the standard bicycle-model reference —
it is the point that travels along the body velocity vector, so no frame conversion is needed
during integration.

**Per step**, given $\Delta t$, the current angle $\delta_k$, a target $\delta^\*$, a rate limit
$\dot{\delta}_{\max}$, and a commanded $v$:

1. **Clamp then ramp.** The target is clamped to the actuator's limits first, then the angle
   advances toward it without overshooting:
   $$\delta_{k+1} = \delta_k + \operatorname{clamp}\!\left(\operatorname{clamp}(\delta^\*,\, \delta_{\min},\, \delta_{\max}) - \delta_k,\; -\dot{\delta}_{\max}\Delta t,\; +\dot{\delta}_{\max}\Delta t\right)$$
   Clamping before ramping means an out-of-range command saturates at the limit rather than
   oscillating around it.
2. **Forward kinematics** with the post-ramp angle gives the body yaw rate:
   $\omega_k = v \tan(\delta_{k+1}) / L$.
3. **Euler pose update**, heading taken at the *start* of the step:
   $$x_{k+1} = x_k + v\cos\theta_k\,\Delta t, \qquad y_{k+1} = y_k + v\sin\theta_k\,\Delta t, \qquad \theta_{k+1} = \operatorname{wrap}(\theta_k + \omega_k \Delta t)$$

Each sample also carries the full `BicycleSteeringState` (four wheel speeds and turning radius)
from running the inverse kinematics on $\omega_k$.

`project(horizon, dt, ...)` repeats this for $\lceil \text{horizon}/\Delta t \rceil$ steps and
returns $\lceil \text{horizon}/\Delta t \rceil + 1$ samples, with the initial state seeded as
element 0 so plots have a clean $t = 0$ anchor.

### Footprints

The projector optionally emits a body polygon per sample, keeping the kinematic model itself
dimension-free. The body is the rectangle spanning
$[-\text{rear\_overhang},\; +\text{front\_overhang}]$ along the heading by
$\text{body\_width}$ across — **both distances measured from the pose reference (the rear
axle)**. Since the front axle sits a full wheelbase ahead of that reference, a physical vehicle
has
$$\text{front\_overhang}_{\text{param}} = L + \text{(overhang past the front axle)}$$
Passing a value smaller than $L$ would place the front bumper *behind* the front axle.

Each polygon is 4 corners, counter-clockwise, **not** closed (the first corner is not repeated),
ordered [rear-right, front-right, front-left, rear-left]. A body width of $0$ or less means
"unset": the footprint comes back empty and projection proceeds normally rather than throwing.

## Future Work
Add diagrams to the readme for visualization
