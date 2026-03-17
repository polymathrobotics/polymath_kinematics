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

## Future Work
Add diagrams to the readme for visualization
