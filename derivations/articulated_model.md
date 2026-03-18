# Articulated Vehicle Kinematics

A vehicle with two rigid sections connected by a central pivot (the articulation joint). Examples: wheel loaders, articulated dump trucks. The vehicle steers by changing the angle between the front and rear sections rather than rotating the wheels.

## Parameters

| Symbol | Code name | Description |
|---|---|---|
| $L_f$ | `articulation_to_front_axle_m` | Distance from articulation joint to front axle |
| $L_r$ | `articulation_to_rear_axle_m` | Distance from articulation joint to rear axle |
| $W_f$ | `front_track_width_m` | Width between the center of the front wheels |
| $W_r$ | `rear_track_width_m` | Width between the center of the rear wheels |
| $r_f$ | `front_wheel_radius_m` | Front wheel radius |
| $r_r$ | `rear_wheel_radius_m` | Rear wheel radius |

## Coordinate convention

- $v$ — forward linear velocity of the vehicle (positive = forward)
- $\omega$ — yaw rate of the body (positive = counter-clockwise / left turn)
- $\gamma$ — articulation angle between front and rear sections (positive = left turn, counterclockwise)
- $\dot{\gamma}$ — articulation angle rate of change (currently fixed at $0$ in the implementation; placeholder for future estimation)
- $\theta_f$ — heading angle of the front body
- $\theta_r$ — heading angle of the rear body

> **Note on sign convention:** The kinematic model below is adapted from Corke & Ridley (2001) [1], which derives the steering kinematics for a center-articulated vehicle. The original paper uses the convention $\theta_f = \theta_r - \gamma$, where a positive articulation angle corresponds to a clockwise (rightward) turn. This paper uses the **ROS2 REP-103 convention** [2], in which the Z-axis points upward and counterclockwise rotations are positive. Accordingly, we use $\theta_f = \theta_r + \gamma$, which flips the sign of the yaw rate equations relative to [1]. All other assumptions from [1] are retained: point wheel–ground contact, no wheel slip, and planar motion.

---

## Derivation of Axle Turning Velocities

This section derives $\omega_{\text{front}} = \dot{\theta}_f$ from first principles, following the methodology of Corke & Ridley [1] with the ROS2 sign convention applied.

### Kinematic equations of the front body

The velocity of the front axle center $P_f$ is:

$$\dot{x}_f = v \cos\theta_f \tag{1}$$

$$\dot{y}_f = v \sin\theta_f \tag{2}$$

### Geometric constraint between bodies

The positions of $P_f$ and $P_r$ are related through the articulation joint $H$:

$$x_r + L_r \cos\theta_r + L_f \cos\theta_f = x_f \tag{3}$$

$$y_r + L_r \sin\theta_r + L_f \sin\theta_f = y_f \tag{4}$$

### Nonholonomic (no-slip) constraints

Rolling without slipping requires zero velocity component along each axle:

$$\dot{x}_r \sin\theta_r - \dot{y}_r \cos\theta_r = 0 \tag{5}$$

$$\dot{x}_f \sin\theta_f - \dot{y}_f \cos\theta_f = 0 \tag{6}$$

Equation (6) is satisfied identically by substituting (1) and (2).

### Time differentiation of geometric constraints

Differentiating (3) and (4) with respect to time and substituting (1) and (2):

$$\dot{x}_r = v\cos\theta_f + L_r \sin\theta_r\,\dot{\theta}_r + L_f \sin\theta_f\,\dot{\theta}_f \tag{7}$$

$$\dot{y}_r = v\sin\theta_f - L_r \cos\theta_r\,\dot{\theta}_r - L_f \cos\theta_f\,\dot{\theta}_f \tag{8}$$

### Applying the rear nonholonomic constraint

Substituting (7) and (8) into (5), expanding, and applying $\sin^2 + \cos^2 = 1$ along with compound angle identities:

$$v\sin(\theta_r - \theta_f) + L_r\,\dot{\theta}_r + L_f\,\dot{\theta}_f\cos(\theta_f - \theta_r) = 0 \tag{9}$$

### Applying the ROS2 angle convention

With $\theta_f = \theta_r + \gamma$ (ROS2 convention):

- $\theta_r - \theta_f = -\gamma \implies \sin(\theta_r - \theta_f) = -\sin\gamma$
- $\theta_f - \theta_r = \gamma \implies \cos(\theta_f - \theta_r) = \cos\gamma$
- Differentiating: $\dot{\theta}_r = \dot{\theta}_f - \dot{\gamma}$

Substituting into (9):

$$-v\sin\gamma + L_r(\dot{\theta}_f - \dot{\gamma}) + L_f\,\dot{\theta}_f\cos\gamma = 0$$

Collecting $\dot{\theta}_f$ terms:

$$\dot{\theta}_f(L_r + L_f\cos\gamma) = v\sin\gamma + L_r\,\dot{\gamma}$$

$$\boxed{\omega_{\text{front}} = \dot{\theta}_f = \frac{v\sin\gamma + L_r\,\dot{\gamma}}{L_f\cos\gamma + L_r}} \tag{10}$$

The rear body yaw rate follows from differentiating the angle convention $\theta_f = \theta_r + \gamma$:

$$\boxed{\omega_{\text{rear}} = \dot{\theta}_r = \dot{\theta}_f - \dot{\gamma} = \frac{v\sin\gamma + L_r\,\dot{\gamma}}{L_f\cos\gamma + L_r} - \dot{\gamma}} \tag{11}$$

> **Comparison with [1]:** The original paper obtains $\dot{\theta}_f = -(v\sin\gamma + l_2\dot{\gamma})/(l_1\cos\gamma + l_2)$. Our result differs only in sign, which is the direct consequence of the flipped angle convention.

---

## Derivation of Body Velocity to Articulation Angle

Given a desired body velocity $(v, \omega)$, we need to find the articulation angle $\gamma$ that produces it.

### Setting up the harmonic addition

Setting $\omega_{\text{front}} = \omega$ in equation (10) and rearranging:

$$v\sin\gamma + L_r\,\dot{\gamma} = \omega(L_f\cos\gamma + L_r)$$

Rearranging to the standard harmonic form $A\cos\gamma + B\sin\gamma$ on the left:

$$\omega L_f\cos\gamma - v\sin\gamma = L_r(\dot{\gamma} - \omega) \tag{12}$$

With $A = \omega L_f$ and $B = -v$, the harmonic addition theorem states:

$$A\cos\gamma + B\sin\gamma = R\cos(\gamma + \phi)$$

where:

$$R = \sqrt{A^2 + B^2} = \sqrt{\omega^2 L_f^2 + v^2}$$

$$\phi = \arctan\!\left(\frac{-B}{A}\right) = \arctan\!\left(\frac{v}{\omega L_f}\right) = \operatorname{atan2}(v,\; \omega L_f)$$

### Solving for $\gamma$

Substituting into (12):

$$R\cos(\gamma + \phi) = L_r(\dot{\gamma} - \omega)$$

$$\cos(\gamma + \phi) = \frac{L_r(\dot{\gamma} - \omega)}{\sqrt{\omega^2 L_f^2 + v^2}}$$

$$\gamma + \phi = \arccos\!\left(\frac{L_r(\dot{\gamma} - \omega)}{\sqrt{\omega^2 L_f^2 + v^2}}\right)$$

$$\boxed{\gamma = \operatorname{sign}(v)\cdot\arccos\!\left(\frac{L_r\,(\dot{\gamma} - \omega)}{\sqrt{\omega^2 L_f^2 + v^2}}\right) - \operatorname{atan2}(v,\; \omega\, L_f)} \tag{13}$$

With $\dot{\gamma} = 0$ (current implementation):

$$\gamma = \operatorname{sign}(v)\cdot\arccos\!\left(\frac{-L_r\,\omega}{\sqrt{\omega^2 L_f^2 + v^2}}\right) - \operatorname{atan2}(v,\; \omega\, L_f) \tag{14}$$

> **Branch selection:** The cosine inversion $\cos(\gamma+\phi) = c$ has two solutions $\gamma = \pm\arccos(c) - \phi$. $\operatorname{sign}(v)$ selects the appropriate branch in both directions.
---

## Turning radii

Once the articulation angle is known, the turning radius for each axle is computed from the triangle formed by the articulation joint and the ICR:

$$R_{\text{front}} = \frac{L_f \cos\gamma + L_r}{\sin\gamma}$$

$$R_{\text{rear}} = \frac{L_r \cos\gamma + L_f}{\sin\gamma}$$

These are the signed distances from the ICR to each axle center, measured along the respective axle lines. See Section II of [1] for the full geometric derivation.

---

## Wheel speeds

Each wheel sits at $\pm$ half the track width from its axle center. Using rigid-body kinematics (ground speed $= \omega \times$ distance from ICR):

$$\omega_{\text{fr}} = \frac{(R_{\text{front}} + W_f/2)\;\omega}{r_f} \qquad \omega_{\text{fl}} = \frac{(R_{\text{front}} - W_f/2)\;\omega}{r_f}$$

$$\omega_{\text{rr}} = \frac{(R_{\text{rear}} + W_r/2)\;\omega}{r_r} \qquad \omega_{\text{rl}} = \frac{(R_{\text{rear}} - W_r/2)\;\omega}{r_r}$$

Outer wheels (further from ICR) spin faster; inner wheels spin slower.

---

## Axle turning velocities

Given the current articulation angle and forward velocity, the turning velocity of each axle can be computed. This is useful for odometry — converting articulation sensor readings back to axle-level yaw rates.

$$\omega_{\text{front}} = \frac{v\sin\gamma + L_r\,\dot{\gamma}}{L_f \cos\gamma + L_r}$$

$$\omega_{\text{rear}} = \omega_{\text{front}} - \dot{\gamma}$$

With $\dot{\gamma} = 0$:

$$\omega_{\text{front}} = \frac{v \sin\gamma}{L_f \cos\gamma + L_r}$$

$$\omega_{\text{rear}} = \omega_{\text{front}}$$

When $\dot{\gamma} = 0$, both axles have the same turning velocity because the articulation angle is not changing — the vehicle is in a steady-state turn.

---

## Roundtrip consistency

Computing `bodyVelocityToVehicleState(v, omega)` to get $\gamma$, then feeding that $\gamma$ into `articulationToAxleVelocities(v, gamma)`, yields $\omega_{\text{front}} = \omega$. The signs are consistent because both the body velocity convention and the axle velocity convention follow ROS2 REP-103 (counterclockwise positive).

---

## Future work

The $\dot{\gamma}$ term is currently hardcoded to $0$. Once articulation angle rate estimation is available (e.g., from an IMU or joint encoder derivative), it will be incorporated to improve accuracy during transient steering maneuvers.

Add diagrams to the readme for visualization

---

## References

[1] P. I. Corke and P. Ridley, "Steering Kinematics for a Center-Articulated Mobile Robot," *IEEE Transactions on Robotics and Automation*, vol. 17, no. 2, pp. 215–218, April 2001.

[2] ROS REP-103, "Standard Units of Measure and Coordinate Conventions," ROS.org, 2010. [Online]. Available: https://www.ros.org/reps/rep-0103.html
