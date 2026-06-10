# SEBA-ROBOT — Dynamics and Velocity-Tracking Control Model

This document describes the nonlinear dynamics, reduced velocity-control state model, linearization, and robust servomechanism LQR controller used for SEBA-ROBOT.

The nonlinear dynamic model follows standard two-wheeled inverted-pendulum robot formulations. The outer-loop control model treats the wheel torques as the effective plant inputs.

---

## 1. Coordinate System and Sign Conventions

The robot is modeled as a two-wheeled inverted pendulum moving on a flat horizontal surface.

The generalized coordinates are

```math
q =
\begin{bmatrix}
x \\
\theta \\
\psi
\end{bmatrix}
```

with

```math
\dot{q} =
\begin{bmatrix}
\dot{x} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
```

and

```math
\ddot{q} =
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}.
```

where:

| Symbol | Meaning |
|---|---|
| `x` | forward position of the wheel axle midpoint |
| `theta` | body pitch angle measured from the upright equilibrium |
| `psi` | yaw angle |
| `x_dot` | forward velocity |
| `theta_dot` | pitch angular velocity |
| `psi_dot` | yaw angular velocity |

The sign conventions are:

| Quantity | Positive direction |
|---|---|
| `x` | forward |
| `theta` | body leans forward from the upright position |
| `psi` | positive yaw according to the right-hand rule about the vertical axis |
| `T_L`, `T_R` | wheel-side torques that drive the corresponding wheel forward |
| `psi_dot` | positive yaw rate according to the sign convention of `psi` |

The wheel-torque input vector is

```math
\tau =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}
```

where `T_L` and `T_R` are wheel-side torques at the wheel axle after gearbox reduction.

---

## 2. Physical Parameters

| Symbol | Meaning |
|---|---|
| `m_p` | body mass |
| `m_w` | mass of one wheel |
| `l_p` | distance from wheel axle to body center of mass |
| `r_w` | wheel radius |
| `W` | wheel separation, measured from left wheel contact center to right wheel contact center |
| `I_px` | body moment of inertia about the body roll axis |
| `I_py` | body moment of inertia about the body pitch axis |
| `I_pz` | body moment of inertia about the body yaw axis |
| `I_w` | wheel moment of inertia about the rolling axis |
| `J_w` | wheel moment of inertia about the vertical/yaw axis |
| `c` | viscous damping coefficient associated with wheel rotation |
| `g` | gravitational acceleration |

The symbol `W` is used for wheel separation to avoid confusion with the differential operator `d/dt`.

---

## 3. Nonlinear Dynamics

The nonlinear equations of motion are written in manipulator form:

```math
M(q)\ddot{q}
+
C_q(q,\dot{q})\dot{q}
+
D_q\dot{q}
+
G(q)
=
B_{\tau}\tau.
```

Solving for acceleration gives

```math
\ddot{q}
=
M(q)^{-1}
\left[
B_{\tau}\tau
-
C_q(q,\dot{q})\dot{q}
-
D_q\dot{q}
-
G(q)
\right].
```

---

## 4. Inertia Matrix

```math
M(q)
=
\begin{bmatrix}
m_p + 2m_w + \frac{2I_w}{r_w^2}
&
m_p l_p \cos\theta
&
0
\\
m_p l_p \cos\theta
&
I_{py} + m_p l_p^2
&
0
\\
0
&
0
&
I_{pz}
+
2J_w
+
\frac{\left(m_w + \frac{I_w}{r_w^2}\right)W^2}{2}
-
\left(I_{pz} - I_{px} - m_p l_p^2\right)\sin^2\theta
\end{bmatrix}.
```

---

## 5. Coriolis Matrix

```math
C_q(q,\dot{q})
=
\begin{bmatrix}
0
&
-m_p l_p \dot{\theta}\sin\theta
&
-m_p l_p \dot{\psi}\sin\theta
\\
0
&
0
&
\left(I_{pz}-I_{px}-m_p l_p^2\right)\dot{\psi}\sin\theta\cos\theta
\\
m_p l_p \dot{\psi}\sin\theta
&
-\left(I_{pz}-I_{px}-m_p l_p^2\right)\dot{\psi}\sin\theta\cos\theta
&
-\left(I_{pz}-I_{px}-m_p l_p^2\right)\dot{\theta}\sin\theta\cos\theta
\end{bmatrix}.
```

---

## 6. Damping Matrix

```math
D_q
=
\begin{bmatrix}
\frac{2c}{r_w^2}
&
-\frac{2c}{r_w}
&
0
\\
-\frac{2c}{r_w}
&
2c
&
0
\\
0
&
0
&
\frac{W^2c}{2r_w^2}
\end{bmatrix}.
```

---

## 7. Gravity Vector

```math
G(q)
=
\begin{bmatrix}
0
\\
-m_p l_p g \sin\theta
\\
0
\end{bmatrix}.
```

With the sign convention used here, a positive forward pitch angle produces an open-loop gravitational tendency to increase the forward pitch angle around the upright equilibrium.

---

## 8. Wheel-Torque Input Mapping

```math
B_{\tau}
=
\begin{bmatrix}
\frac{1}{r_w}
&
\frac{1}{r_w}
\\
-1
&
-1
\\
-\frac{W}{2r_w}
&
\frac{W}{2r_w}
\end{bmatrix}.
```

The first row maps wheel torques into forward generalized force.

The second row represents the reaction torque applied to the body pitch coordinate by the driven wheels.

The third row maps the left-right wheel torque difference into yaw generalized torque. The use of `W/(2r_w)` assumes that `W` is the full wheel separation.

---

## 9. Nonlinear State Model

Define

```math
q =
\begin{bmatrix}
x \\
\theta \\
\psi
\end{bmatrix},
\qquad
\dot{q} =
\begin{bmatrix}
\dot{x} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix},
\qquad
\tau =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}.
```

Then

```math
\ddot{q}
=
f(q,\dot{q},\tau).
```

Component-wise,

```math
\begin{aligned}
\ddot{x}
&=
f_1(\dot{x},\theta,\dot{\theta},\dot{\psi},T_L,T_R),
\\
\ddot{\theta}
&=
f_2(\dot{x},\theta,\dot{\theta},\dot{\psi},T_L,T_R),
\\
\ddot{\psi}
&=
f_3(\dot{x},\theta,\dot{\theta},\dot{\psi},T_L,T_R).
\end{aligned}
```

For velocity control, absolute position `x` and absolute yaw `psi` are omitted because the model is invariant to both coordinates.

---

## 10. Reduced Velocity-Control State

The reduced state vector is

```math
X =
\begin{bmatrix}
v \\
\theta \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
```

where

```math
v
=
\dot{x}.
```

The reduced state derivative is

```math
\dot{X}
=
\begin{bmatrix}
\dot{v} \\
\dot{\theta} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}.
```

Using the nonlinear acceleration model:

```math
\dot{X}
=
F(X,U)
```

where

```math
U =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}
```

and

```math
F(X,U)
=
\begin{bmatrix}
f_1(X,U)
\\
\dot{\theta}
\\
f_2(X,U)
\\
f_3(X,U)
\end{bmatrix}.
```

---

## 11. Linearization

The system is linearized around the upright stationary equilibrium:

```math
v = 0,
\qquad
\theta = 0,
\qquad
\dot{\theta} = 0,
\qquad
\dot{\psi} = 0,
\qquad
T_L = 0,
\qquad
T_R = 0.
```

with small-angle approximations:

```math
\sin\theta \approx \theta,
\qquad
\cos\theta \approx 1.
```

The linearized plant is

```math
\dot{X}
=
AX
+
BU
```

where

```math
A
=
\left.
\frac{\partial F}{\partial X}
\right|_{\mathrm{eq}},
\qquad
B
=
\left.
\frac{\partial F}{\partial U}
\right|_{\mathrm{eq}}.
```

The state and input are

```math
X =
\begin{bmatrix}
v \\
\theta \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix},
\qquad
U =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}.
```

The second state equation is kinematic:

```math
\frac{d\theta}{dt}
=
\dot{\theta}.
```

Therefore, the second row of the linearized matrices is

```math
A_2
=
\begin{bmatrix}
0 & 0 & 1 & 0
\end{bmatrix},
\qquad
B_2
=
\begin{bmatrix}
0 & 0
\end{bmatrix}.
```

---

## 12. Output Definition

The velocity-control mode tracks forward velocity and yaw rate.

The output vector is

```math
Y =
\begin{bmatrix}
v \\
\dot{\psi}
\end{bmatrix}.
```

The output equation is

```math
Y
=
CX
+
DU
```

with

```math
C
=
\begin{bmatrix}
1 & 0 & 0 & 0
\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
D
=
\begin{bmatrix}
0 & 0
\\
0 & 0
\end{bmatrix}.
```

---

## 13. Robust Servomechanism LQR Formulation

The nominal linear plant is

```math
\dot{X}
=
AX
+
BU,
\qquad
Y
=
CX
+
DU.
```

Unmodeled effects may be represented as bounded disturbances, but the nominal RSLQR gain is designed using `(A, B, C, D)`.

The control objective is upright stabilization with forward-velocity and yaw-rate tracking.

The output and constant command are

```math
Y =
\begin{bmatrix}
v
\\
\dot{\psi}
\end{bmatrix},
\qquad
r =
\begin{bmatrix}
v_{\mathrm{cmd}}
\\
\dot{\psi}_{\mathrm{cmd}}
\end{bmatrix}.
```

The tracking error is defined as

```math
e
=
Y
-
r.
```

For constant command `r`,

```math
\dot{e}
=
\dot{Y}.
```

Using

```math
Y
=
CX
+
DU,
```

we obtain

```math
\dot{e}
=
C\dot{X}
+
D\dot{U}.
```

Following the robust servomechanism formulation, define the augmented state

```math
Z =
\begin{bmatrix}
e
\\
\dot{X}
\end{bmatrix}
```

and define the augmented control input as the derivative of the plant input:

```math
\mu
=
\dot{U}.
```

Since

```math
\ddot{X}
=
A\dot{X}
+
B\dot{U}
```

for the nominal linear plant, the augmented model becomes

```math
\dot{Z}
=
A_{\mathrm{aug}}Z
+
B_{\mathrm{aug}}\mu
```

with

```math
A_{\mathrm{aug}}
=
\begin{bmatrix}
0_{2 \times 2}
&
C
\\
0_{4 \times 2}
&
A
\end{bmatrix},
\qquad
B_{\mathrm{aug}}
=
\begin{bmatrix}
D
\\
B
\end{bmatrix}.
```

Because `D = 0` for this robot model,

```math
B_{\mathrm{aug}}
=
\begin{bmatrix}
0_{2 \times 2}
\\
B
\end{bmatrix}.
```

The augmented state dimension is

```math
Z
\in
\mathbb{R}^{6}
```

and the augmented input dimension is

```math
\mu
\in
\mathbb{R}^{2}.
```

---

## 14. RSLQR Control Law

The LQR cost function is

```math
J
=
\int_{0}^{\infty}
\left(
Z^TQZ
+
\mu^TR\mu
\right)
dt
```

where:

| Matrix | Requirement |
|---|---|
| `Q` | symmetric positive semidefinite |
| `R` | symmetric positive definite |

The gain matrix is obtained from the continuous-time algebraic Riccati equation:

```math
PA_{\mathrm{aug}}
+
A_{\mathrm{aug}}^TP
-
PB_{\mathrm{aug}}R^{-1}B_{\mathrm{aug}}^TP
+
Q
=
0.
```

Then

```math
K_c
=
R^{-1}B_{\mathrm{aug}}^TP.
```

The control law is

```math
\mu
=
-
K_cZ.
```

Since

```math
\mu
=
\dot{U},
```

the RSLQR output is the wheel-torque derivative. The wheel-torque command is obtained by integration:

```math
U(t)
=
U(0)
+
\int_{0}^{t}
\dot{U}(\tau)d\tau.
```

In discrete time:

```math
U[k+1]
=
U[k]
+
T_s\mu[k]
```

where `T_s` is the controller sample time.

The resulting torque command is

```math
U =
\begin{bmatrix}
T_L
\\
T_R
\end{bmatrix}.
```

---

## 15. Input Constraints

The wheel-torque command obtained by integrating `U_dot` is limited by the physical actuator range:

```math
|T_L|
\leq
T_{\max,L},
\qquad
|T_R|
\leq
T_{\max,R}.
```

For symmetric limits,

```math
|T_L|
\leq
T_{\max},
\qquad
|T_R|
\leq
T_{\max}.
```

When a torque component reaches its limit, only torque-rate commands that drive it back toward the admissible range are integrated. This keeps the commanded wheel torque physically feasible.

---

## 16. Actuator Assumption and Torque Tracking

The actuator subsystem is modeled separately from the outer-loop dynamics. The motor electrical dynamics and PWM generation are handled by a faster inner current-control loop.

Under the cascaded-control assumption, the current-loop bandwidth is sufficiently higher than the outer balance-control bandwidth, so the commanded wheel torque is treated as tracked with small delay.

For a DC motor,

```math
T
\approx
K_tI
```

so the wheel-torque command is converted to a current command by

```math
I_{\mathrm{cmd}}
=
\frac{T_{\mathrm{cmd}}}{K_t}.
```

If this assumption is not valid, actuator dynamics may be added later, for example:

```math
\dot{T}_{\mathrm{actual}}
=
\frac{
T_{\mathrm{cmd}}
-
T_{\mathrm{actual}}
}{
\tau_m
}.
```

---

## References

1. S. Kim and S. J. Kwon, “Dynamic Modeling of a Two-wheeled Inverted Pendulum Balancing Mobile Robot,” *International Journal of Control, Automation and Systems*, vol. 13, no. 4, pp. 926–933, 2015.

2. S. Firuzi and S. Gong, “Attitude Control of a Flexible Solar Sail in Low Earth Orbit,” *Journal of Guidance, Control, and Dynamics*, vol. 41, no. 8, pp. 1715–1730, 2018.

3. E. Lavretsky and K. A. Wise, *Robust and Adaptive Control: With Aerospace Applications*, Springer, 2013.
