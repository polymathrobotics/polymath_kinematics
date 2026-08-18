# Differential Drive Kinematics

A two-wheeled robot where each wheel is independently driven. The wheels share a common axle and the robot pivots about a point on that axle.

## Parameters

| Symbol | Code name | Description |
|---|---|---|
| $r$ | `wheel_radius_m` | Wheel radius |
| $W$ | `track_width_m` | Distance between the center of the two wheels |

## Coordinate convention

- $v$ — forward linear velocity of the body center (positive = forward)
- $\omega$ — angular velocity about the body center (positive = counter-clockwise / left turn)
- $\omega_L, \omega_R$ — angular velocities of the left and right wheels (positive = forward rolling)

## Forward kinematics (wheel velocities → body velocity)

Each wheel's ground speed is its angular velocity times the wheel radius:

$$v_L = \omega_L \cdot r \qquad v_R = \omega_R \cdot r$$

The body center lies midway between the wheels, so its linear velocity is the average of the two ground speeds:

$$v = \frac{v_L + v_R}{2} = \frac{r\,(\omega_L + \omega_R)}{2}$$

The angular velocity comes from the velocity difference across the track width. The right wheel is at $+W/2$ from center and the left at $-W/2$, so:

$$\omega = \frac{v_R - v_L}{W} = \frac{r\,(\omega_R - \omega_L)}{W}$$

## Inverse kinematics (body velocity → wheel velocities)

Solve the forward equations for the wheel ground speeds:

$$v_L = v - \omega \cdot \frac{W}{2} \qquad v_R = v + \omega \cdot \frac{W}{2}$$

Then convert to wheel angular velocities:

$$\omega_L = \frac{v_L}{r} = \frac{v - \omega \cdot W/2}{r}$$

$$\omega_R = \frac{v_R}{r} = \frac{v + \omega \cdot W/2}{r}$$

## Special cases

| Condition | Behavior |
|---|---|
| $\omega_L = \omega_R$ | Straight line ($\omega = 0$) |
| $\omega_L = -\omega_R$ | Spin in place ($v = 0$), ICR at body center |
| One wheel stationary | Pivot about that wheel |

## Instantaneous center of rotation (ICR)

The ICR lies on the wheel axle at a signed distance from the body center:

$$d_{\text{ICR}} = \frac{v}{\omega} = \frac{W}{2} \cdot \frac{\omega_R + \omega_L}{\omega_R - \omega_L}$$

When the robot drives straight, the ICR is at infinity. When spinning in place, it is at the origin.

## Forward projection (`DifferentialDriveProjector`)

`DifferentialDriveProjector` wraps this model to roll a pose forward in time under acceleration
limits. Unlike the bicycle and articulated projectors — which rate-limit a steering *angle* — a
differential drive has no steered joint, so what is limited here is the **body command itself**:
$v$ and $\omega$ each ramp toward their target under their own acceleration limit.

**Pose reference.** The pose is the **body centre** (midway between the two wheels), the point
the ICR distance is measured from above.

**Per step**, given $\Delta t$, current $(v_k, \omega_k)$, targets $(v^\*, \omega^\*)$, and
limits $(a_{\max}, \alpha_{\max})$:

1. **Clamp then ramp**, independently per channel, neither overshooting:
   $$v_{k+1} = v_k + \operatorname{clamp}\!\left(\operatorname{clamp}(v^\*,\, v_{\min},\, v_{\max}) - v_k,\; -a_{\max}\Delta t,\; +a_{\max}\Delta t\right)$$
   $$\omega_{k+1} = \omega_k + \operatorname{clamp}\!\left(\operatorname{clamp}(\omega^\*,\, \omega_{\min},\, \omega_{\max}) - \omega_k,\; -\alpha_{\max}\Delta t,\; +\alpha_{\max}\Delta t\right)$$
   Clamping before ramping means an out-of-range command saturates at the limit rather than
   oscillating around it.
2. **Inverse kinematics** on the post-ramp command gives the wheel speeds recorded on the sample.
3. **Euler pose update**, using the post-ramp velocities with the heading taken at the *start* of
   the step:
   $$x_{k+1} = x_k + v_{k+1}\cos\theta_k\,\Delta t, \qquad y_{k+1} = y_k + v_{k+1}\sin\theta_k\,\Delta t, \qquad \theta_{k+1} = \operatorname{wrap}(\theta_k + \omega_{k+1} \Delta t)$$

`project(horizon, dt, ...)` repeats this for $\lceil \text{horizon}/\Delta t \rceil$ steps and
returns $\lceil \text{horizon}/\Delta t \rceil + 1$ samples, with the initial state seeded as
element 0 so plots have a clean $t = 0$ anchor. Setting initial $=$ target makes the ramp a
no-op, which is how a constant-command trajectory (e.g. one lattice cell) is generated.

### Footprints

The projector optionally emits a body polygon per sample, keeping the kinematic model itself
dimension-free. The footprint is an **arbitrary polygon** supplied in the body frame: $+x$ along the
heading, $+y$ to the left, origin at the body centre. Each sample carries it transformed into the
world frame by that sample's pose.

A differential drive has a single axle, so unlike the bicycle and articulated projectors there is no
axle reference to select — the body centre is the only sensible origin, and polygon extents are the
bumper distances directly.

`rectangleFootprint(front_m, rear_m, width_m)` builds the boxy case, ordered counter-clockwise and
open: rear-right, front-right, front-left, rear-left. An empty polygon means "unset": the footprint
comes back empty and projection proceeds normally rather than throwing.

## Future Work

Add diagrams to the readme for visualization
