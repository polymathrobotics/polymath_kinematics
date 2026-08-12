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

**Pose reference.** Poses are measured at whichever axle the caller selects
(`AxleReference::FRONT` or `REAR`). Motion is always integrated at the **rear axle**, the classic
bicycle reference and the point that travels along the body velocity vector. Because the chassis is
one rigid body, both axles share the heading $\theta$, so a FRONT reference is a pure longitudinal
offset applied on input and output:

$$\mathbf{p}^{\text{front}} = \mathbf{p}^{\text{rear}} + L\begin{bmatrix}\cos\theta\\\sin\theta\end{bmatrix}$$

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
dimension-free. The footprint is an **arbitrary polygon** supplied in the body frame of the selected
axle: $+x$ along the chassis heading, $+y$ to the left, origin at that axle. Each sample carries the
polygon transformed into the world frame by that sample's pose.

Because the reference axle sets the polygon's origin, the same physical body is described
differently depending on the choice. For a vehicle with $f$ of overhang past the front axle and $r$
of tail behind the rear axle:

| Reference | Forward extent | Rearward extent |
|---|---|---|
| `REAR` | $L + f$ | $r$ |
| `FRONT` | $f$ | $L + r$ |

`rectangleFootprint(front_m, rear_m, width_m)` builds the boxy case, ordered counter-clockwise and
open: rear-right, front-right, front-left, rear-left. An empty polygon means "unset": the footprint
comes back empty and projection proceeds normally rather than throwing.

## Future Work
Add diagrams to the readme for visualization
