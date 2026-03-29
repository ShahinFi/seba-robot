# SEBA-BOT — Dynamics and State-Space Model (Velocity Control Mode)

## 1. Generalized Coordinates

The robot is described by the generalized coordinates

q   = [ x, tp, tb ]ᵀ
q̇  = [ ẋ, d(tp)/dt, d(tb)/dt ]ᵀ
q̈  = [ ẍ, d²(tp)/dt², d²(tb)/dt² ]ᵀ

where:

- x  : forward position
- tp : pitch angle
- tb : yaw angle

The control inputs are the wheel torques

τ = [ TL, TR ]ᵀ

---

## 2. Nonlinear Dynamics

M(q) q̈ + C_q(q, q̇) q̇ + D q̇ + G(q) = B_τ τ

---

### Inertia Matrix

```text
M =
[ mp + 2mw + 2Iw/rw²        mp*lp*cos(tp)                                      0
  mp*lp*cos(tp)             Ipy + mp*lp²                                       0
  0                         0      Ipz + 2Jw + ((mw + Iw/rw²)*d²)/2 - (Ipz - Ipx - mp*lp²)*sin(tp)² ]
```

---

### Coriolis Matrix

```text
C_q =
[ 0                         -mp*lp*(d(tp)/dt)*sin(tp)         -mp*lp*(d(tb)/dt)*sin(tp)
  0                          0                                 (Ipz - Ipx - mp*lp²)*(d(tb)/dt)*sin(tp)*cos(tp)
  mp*lp*(d(tb)/dt)*sin(tp) -(Ipz - Ipx - mp*lp²)*(d(tb)/dt)*sin(tp)*cos(tp)   -(Ipz - Ipx - mp*lp²)*(d(tp)/dt)*sin(tp)*cos(tp) ]
```

---

### Damping Matrix

```text
D =
[  2*c/rw²   -2*c/rw    0
  -2*c/rw     2*c       0
   0          0         (d²/(2*rw²))*c ]
```

---

### Gravity Vector

```text
G =
[ 0
  -mp*lp*g*sin(tp)
  0 ]
```

---

### Input Mapping Matrix

```text
B_τ =
[  1/rw      1/rw
  -1        -1
  -d/(2*rw)  d/(2*rw) ]
```

---

## 3. Explicit Nonlinear Acceleration Model

q̈ = M⁻¹ * ( B_τ τ - C_q q̇ - D q̇ - G )

which gives

- ẍ = f1(ẋ, tp, d(tp)/dt, d(tb)/dt, TL, TR)
- d²(tp)/dt² = f2(...)
- d²(tb)/dt² = f3(...)

---

## 4. Reduced State (Velocity Control Mode)

X = [ ẋ, tp, d(tp)/dt, d(tb)/dt ]ᵀ

Ẋ = [ ẍ, d(tp)/dt, d²(tp)/dt², d²(tb)/dt² ]ᵀ

---

## 5. Linearization

Equilibrium:

ẋ = 0
tp = 0
d(tp)/dt = 0
d(tb)/dt = 0
TL = 0
TR = 0

Small-angle:

sin(tp) ≈ tp
cos(tp) ≈ 1

---

## 6. Output

```text
C =
[ 1  0  0  0
  0  0  0  1 ]
```

Y = [ ẋ, d(tb)/dt ]ᵀ

---

## 7. State Space

U = [ TL, TR ]ᵀ

```text
A =
[ A1
  A2
  A3
  A4 ]

B =
[ B1
  B2
  B3
  B4 ]
```

with

A2 = [ 0  0  1  0 ]
B2 = [ 0  0 ]

---

## 8. Augmented Servo Model

```text
X̃ =
[ integral(ẋ - x_dot_command)
  integral(d(tb)/dt - tb_dot_command)
  ẋ
  tp
  d(tp)/dt
  d(tb)/dt ]
```

```text
Ã =
[ 0_(2x2)   C_(2x4)
  0_(4x2)   A_(4x4) ]

B̃ =
[ 0_(2x2)
  B_(4x2) ]
```

---

## 9. Control Law

U = -Kc * X̃

---

## 10. Summary

1. Full nonlinear dynamics
2. Solve accelerations
3. Define reduced state
4. Linearize
5. Define outputs
6. Augment with integrators
7. Apply LQR

Final system: augmented 6-state velocity tracking model