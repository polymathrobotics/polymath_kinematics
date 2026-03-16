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

## Future Work

Add diagrams to the readme for visualization
