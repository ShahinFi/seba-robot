# Dynamics and Velocity-Tracking Control Model

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

The generalized velocity and acceleration vectors are

```math
\dot{q} =
\begin{bmatrix}
\dot{x} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix},
\qquad
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
| $x$ | forward position of the wheel axle midpoint |
| $\theta$ | body pitch angle measured from the upright equilibrium |
| $\psi$ | yaw angle |
| $\dot{x}$ | forward velocity |
| $\dot{\theta}$ | pitch angular velocity |
| $\dot{\psi}$ | yaw angular velocity |
| $\ddot{x}$ | forward acceleration |
| $\ddot{\theta}$ | pitch angular acceleration |
| $\ddot{\psi}$ | yaw angular acceleration |

The wheel-torque input vector is

```math
\tau =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}
```

where:

| Symbol | Meaning |
|---|---|
| $T_L$ | left wheel-side torque at the wheel axle after gearbox reduction |
| $T_R$ | right wheel-side torque at the wheel axle after gearbox reduction |

The sign conventions are:

| Quantity | Positive direction |
|---|---|
| $x$ | forward motion of the wheel axle midpoint |
| $\theta$ | body leans forward from the upright position |
| $\psi$ | yaw according to the right-hand rule about the vertical axis |
| $T_L$ | torque that drives the left wheel forward |
| $T_R$ | torque that drives the right wheel forward |

Positive derivatives indicate increasing values of the corresponding coordinates. With this convention, equal positive wheel torques produce forward generalized force, while a left-right torque difference produces yaw generalized torque.

---

## 2. Physical Parameters

| Symbol | Meaning |
|---|---|
| $m_p$ | body mass |
| $m_w$ | mass of one wheel |
| $l_p$ | distance from wheel axle to body center of mass |
| $r_w$ | wheel radius |
| $W$ | wheel separation, measured from left wheel contact center to right wheel contact center |
| $I_{px}$ | body moment of inertia about the body roll axis |
| $I_{py}$ | body moment of inertia about the body pitch axis |
| $I_{pz}$ | body moment of inertia about the body yaw axis |
| $I_w$ | wheel moment of inertia about the rolling axis |
| $J_w$ | wheel moment of inertia about the vertical/yaw axis |
| $c$ | viscous damping coefficient associated with wheel rotation |
| $g$ | gravitational acceleration |

The symbol $W$ is used for wheel separation to avoid confusion with the differential operator $d/dt$.

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

The third row maps the left-right wheel torque difference into yaw generalized torque. The use of $W/(2r_w)$ assumes that $W$ is the full wheel separation.

---

## 9. State-Space Model for Control

For controller design, the nonlinear dynamics are converted into a reduced state-space model and then linearized around the upright stationary operating point:

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

Here, $X$ is the state vector, $U$ is the plant input vector, and $Y$ is the output vector. The matrices $A$, $B$, $C$, and $D$ describe the linearized plant around the chosen operating point.

For velocity control, the absolute position $x$ and absolute yaw angle $\psi$ are omitted because the dynamics depend on their derivatives, not on their absolute values. The reduced state and input are chosen as

```math
X =
\begin{bmatrix}
v \\
\theta \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix},
\qquad
v = \dot{x},
\qquad
U =
\begin{bmatrix}
T_L \\
T_R
\end{bmatrix}.
```

The corresponding state derivative is

```math
\dot{X}
=
\begin{bmatrix}
\dot{v} \\
\dot{\theta} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
\ddot{x} \\
\dot{\theta} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}.
```

The acceleration terms are obtained by solving the nonlinear dynamics for $\ddot{q}$. Therefore, before linearization, the reduced nonlinear plant may be written compactly as

```math
\dot{X}
=
F(X,U).
```

The plant is linearized around the upright stationary equilibrium

```math
X_{\mathrm{eq}}
=
\begin{bmatrix}
0 \\
0 \\
0 \\
0
\end{bmatrix},
\qquad
U_{\mathrm{eq}}
=
\begin{bmatrix}
0 \\
0
\end{bmatrix}.
```

Equivalently,

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

Using the small-angle approximations

```math
\sin\theta \approx \theta,
\qquad
\cos\theta \approx 1,
```

the state matrices are defined by the Jacobians

```math
A
=
\left.
\frac{\partial F}{\partial X}
\right|_{X_{\mathrm{eq}},U_{\mathrm{eq}}},
\qquad
B
=
\left.
\frac{\partial F}{\partial U}
\right|_{X_{\mathrm{eq}},U_{\mathrm{eq}}}.
```

Evaluating these Jacobians gives the following compact symbolic state and input matrices. The symbolic computation used to derive these matrices is included in Appendix A for reproducibility.

Define the common denominator for the coupled forward-pitch dynamics as

```math
\Delta
=
2I_{py}I_w
+
2I_wl_p^2m_p
+
I_{py}m_pr_w^2
+
2I_{py}m_wr_w^2
+
2l_p^2m_pm_wr_w^2.
```

Define the common denominator for the yaw dynamics as

```math
\Gamma
=
I_wW^2
+
2I_{pz}r_w^2
+
4J_wr_w^2
+
W^2m_wr_w^2.
```

Then the linearized state matrix is

```math
A
=
\begin{bmatrix}
A_{11} & A_{12} & A_{13} & 0
\\
0 & 0 & 1 & 0
\\
A_{31} & A_{32} & A_{33} & 0
\\
0 & 0 & 0 & A_{44}
\end{bmatrix}
```

where

```math
A_{11}
=
-\frac{
2c\left(m_pl_p^2 + m_pr_wl_p + I_{py}\right)
}{
\Delta
},
```

```math
A_{12}
=
-\frac{
gl_p^2m_p^2r_w^2
}{
\Delta
},
```

```math
A_{13}
=
\frac{
2cr_w\left(m_pl_p^2 + m_pr_wl_p + I_{py}\right)
}{
\Delta
},
```

```math
A_{31}
=
\frac{
4I_wc
+
2cm_pr_w^2
+
4cm_wr_w^2
+
2cl_pm_pr_w
}{
r_w\Delta
},
```

```math
A_{32}
=
\frac{
gl_pm_p\left(2I_w + m_pr_w^2 + 2m_wr_w^2\right)
}{
\Delta
},
```

```math
A_{33}
=
-\frac{
2c\left(2I_w + m_pr_w^2 + 2m_wr_w^2 + l_pm_pr_w\right)
}{
\Delta
},
```

and

```math
A_{44}
=
-\frac{
cW^2
}{
\Gamma
}.
```

The linearized input matrix is

```math
B
=
\begin{bmatrix}
B_{11} & B_{11}
\\
0 & 0
\\
B_{31} & B_{31}
\\
-B_{41} & B_{41}
\end{bmatrix}
```

where

```math
B_{11}
=
\frac{
r_w\left(m_pl_p^2 + m_pr_wl_p + I_{py}\right)
}{
\Delta
},
```

```math
B_{31}
=
-\frac{
2I_w + m_pr_w^2 + 2m_wr_w^2 + l_pm_pr_w
}{
\Delta
},
```

and

```math
B_{41}
=
\frac{
Wr_w
}{
\Gamma
}.
```

The first and third rows of $B$ have identical left and right torque columns, so the forward and pitch dynamics are driven by the equal-torque component $T_L + T_R$. The fourth row has opposite signs, so the yaw dynamics are driven by the differential-torque component $T_R - T_L$.

The output is chosen as forward velocity and yaw rate:

```math
Y =
\begin{bmatrix}
v \\
\dot{\psi}
\end{bmatrix}.
```

Because the outputs are directly selected from the state and do not depend directly on the wheel-torque input,

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

## 10. Robust Servomechanism LQR Formulation

The RSLQR controller is designed using the nominal linear plant obtained from the linearized dynamics. Unmodeled effects may be represented as bounded disturbances, but the nominal RSLQR gain is designed using the matrices $A$, $B$, $C$, and $D$.

The control objective is upright stabilization with forward-velocity and yaw-rate tracking. Let the constant command vector be

```math
r =
\begin{bmatrix}
v_{\mathrm{cmd}} \\
\dot{\psi}_{\mathrm{cmd}}
\end{bmatrix}.
```

To construct the augmented model, the derivative of the tracking error is needed. The tracking error is

```math
e
=
Y
-
r.
```

Since the command $r$ is assumed constant,

```math
\dot{e}
=
\dot{Y}.
```

Using the output equation,

```math
Y
=
CX
+
DU,
```

the error dynamics become

```math
\dot{e}
=
C\dot{X}
+
D\dot{U}.
```

The robust servomechanism design model augments the tracking error with the state derivative:

```math
Z =
\begin{bmatrix}
e
\\
\dot{X}
\end{bmatrix},
\qquad
\mu
=
\dot{U}.
```

Differentiating the linearized state equation gives

```math
\ddot{X}
=
A\dot{X}
+
B\dot{U}.
```

Thus, the augmented model is

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

Since $D=0$ for this output choice,

```math
B_{\mathrm{aug}}
=
\begin{bmatrix}
0_{2 \times 2}
\\
B
\end{bmatrix}.
```

The augmented state and input dimensions are

```math
Z \in \mathbb{R}^{6},
\qquad
\mu \in \mathbb{R}^{2}.
```

---

## 11. RSLQR Control Law

The LQR cost function for the augmented system is

```math
J
=
\int_{0}^{\infty}
\left(
Z^TQZ
+
\mu^TR\mu
\right)
dt.
```

where:

| Matrix | Requirement |
|---|---|
| $Q$ | symmetric positive semidefinite |
| $R$ | symmetric positive definite |

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
T_s\mu[k].
```

where $T_s$ is the controller sample time.

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

## 12. Input Constraints

The wheel-torque command obtained by integrating $\dot{U}$ is limited by the physical actuator range:

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

## 13. Actuator Assumption and Torque Tracking

The actuator subsystem is modeled separately from the outer-loop dynamics. The motor electrical dynamics and PWM generation are handled by a faster inner current-control loop.

Under the cascaded-control assumption, the current-loop bandwidth is sufficiently higher than the outer balance-control bandwidth, so the commanded wheel torque is treated as tracked with small delay.

For a DC motor,

```math
T
\approx
K_t I
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

---

## Appendix A. MATLAB Symbolic Derivation of the Linearized Matrices

This appendix gives the MATLAB symbolic workflow used to derive the linearized matrices $A$ and $B$ shown in Section 9.

The code starts from the nonlinear dynamics already defined in this document. It solves the nonlinear equations for the generalized accelerations, constructs the reduced nonlinear state derivative, computes the Jacobians, and evaluates them at the upright stationary equilibrium.

### A.1 Symbolic Variables

```matlab
syms mp mw Iw Jw Ipx Ipy Ipz rw W lp c g
syms x x_dot x_dot_dot
syms theta theta_dot theta_dot_dot
syms psi psi_dot psi_dot_dot
syms TL TR
```

### A.2 Nonlinear Model Matrices

```matlab
M11 = mp + 2*mw + 2*Iw/(rw^2);
M12 = mp*lp*cos(theta);
M13 = 0;

M21 = M12;
M22 = Ipy + mp*(lp^2);
M23 = 0;

M31 = 0;
M32 = 0;
M33 = Ipz ...
    + 2*Jw ...
    + (mw + Iw/(rw^2))*(W^2)/2 ...
    - (Ipz - Ipx - mp*(lp^2))*sin(theta)^2;

M = [
    M11, M12, M13;
    M21, M22, M23;
    M31, M32, M33
];
```

```matlab
C11 = 0;
C12 = -mp*lp*theta_dot*sin(theta);
C13 = -mp*lp*psi_dot*sin(theta);

C21 = 0;
C22 = 0;
C23 = (Ipz - Ipx - mp*(lp^2))*psi_dot*sin(theta)*cos(theta);

C31 = -C13;
C32 = -C23;
C33 = -(Ipz - Ipx - mp*(lp^2))*theta_dot*sin(theta)*cos(theta);

Cq = [
    C11, C12, C13;
    C21, C22, C23;
    C31, C32, C33
];
```

```matlab
D11 = 2*c/(rw^2);
D12 = -2*c/rw;
D13 = 0;

D21 = D12;
D22 = 2*c;
D23 = 0;

D31 = 0;
D32 = 0;
D33 = (W^2*c)/(2*(rw^2));

Dq = [
    D11, D12, D13;
    D21, D22, D23;
    D31, D32, D33
];
```

```matlab
G = [
    0;
    -mp*lp*g*sin(theta);
    0
];

Btau = [
    1/rw,       1/rw;
    -1,         -1;
    -W/(2*rw),  W/(2*rw)
];

tau = [
    TL;
    TR
];

q_dot = [
    x_dot;
    theta_dot;
    psi_dot
];

q_dot_dot = [
    x_dot_dot;
    theta_dot_dot;
    psi_dot_dot
];
```

The symbolic nonlinear equation is formed as

```math
M(q)\ddot{q}
+
C_q(q,\dot{q})\dot{q}
+
D_q\dot{q}
+
G(q)
-
B_{\tau}\tau
=
0.
```

```matlab
Eq = M*q_dot_dot + Cq*q_dot + Dq*q_dot + G - Btau*tau;
```

### A.3 Solve for Generalized Accelerations

```matlab
solutions = solve( ...
    [Eq(1) == 0, Eq(2) == 0, Eq(3) == 0], ...
    [x_dot_dot, theta_dot_dot, psi_dot_dot] ...
);

solution_x_dot_dot     = simplify(solutions.x_dot_dot);
solution_theta_dot_dot = simplify(solutions.theta_dot_dot);
solution_psi_dot_dot   = simplify(solutions.psi_dot_dot);
```

### A.4 Construct the Reduced Nonlinear State Derivative

The reduced control state is

```math
X
=
\begin{bmatrix}
\dot{x}
\\
\theta
\\
\dot{\theta}
\\
\dot{\psi}
\end{bmatrix},
\qquad
U
=
\begin{bmatrix}
T_L
\\
T_R
\end{bmatrix}.
```

```matlab
X = [
    x_dot;
    theta;
    theta_dot;
    psi_dot
];

U = [
    TL;
    TR
];

F = [
    solution_x_dot_dot;
    theta_dot;
    solution_theta_dot_dot;
    solution_psi_dot_dot
];
```

### A.5 Compute the Jacobians

```matlab
A = simplify(jacobian(F, X));
B = simplify(jacobian(F, U));
```

### A.6 Evaluate at the Upright Stationary Equilibrium

The Jacobians are evaluated at

```math
\dot{x} = 0,
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

```matlab
eq_vars = [
    x_dot;
    theta;
    theta_dot;
    psi_dot;
    TL;
    TR
];

eq_vals = [
    0;
    0;
    0;
    0;
    0;
    0
];

A = simplify(subs(A, eq_vars, eq_vals));
B = simplify(subs(B, eq_vars, eq_vals));
```

### A.7 Display the Final Symbolic Matrices

```matlab
A
B
```

The resulting symbolic matrices are the compact linearized matrices shown in Section 9. Physical parameter values can be substituted after this step to obtain the numerical state-space model used for simulation or LQR gain computation.

---

## References

1. S. Kim and S. J. Kwon, “Dynamic Modeling of a Two-wheeled Inverted Pendulum Balancing Mobile Robot,” *International Journal of Control, Automation and Systems*, vol. 13, no. 4, pp. 926–933, 2015.

2. S. Firuzi and S. Gong, “Attitude Control of a Flexible Solar Sail in Low Earth Orbit,” *Journal of Guidance, Control, and Dynamics*, vol. 41, no. 8, pp. 1715–1730, 2018.

3. E. Lavretsky and K. A. Wise, *Robust and Adaptive Control: With Aerospace Applications*, Springer, 2013.
