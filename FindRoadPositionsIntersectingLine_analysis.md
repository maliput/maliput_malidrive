# `FindRoadPositionsIntersectingLine` — Algorithm Analysis

## Overview

`FindRoadPositionsIntersectingLine` finds where a 3D line intersects the road surface.
It generalizes `DoFindRoadPositionsAtXY` (which handles the special case of a vertical line at a given XY) to an arbitrary 3D direction.

---

## The Problem

We want to find where a 3D line pierces the road surface. The line is defined as:

$$\text{point}(t) = \text{origin} + t \cdot \text{direction}$$

The road surface is parameterized by the `RoadCurve` mapping `W(p, r, 0)`, which maps road-curve coordinates `(p, r)` to 3D world coordinates `(x, y, z)`, where `h=0` means "on the surface."

An intersection exists when:

$$W(p, r, 0) = \text{origin} + t \cdot \text{direction}$$

Rearranging, we define the residual function:

$$F(p, r, t) = W(p, r, 0) - \text{origin} - t \cdot \text{direction} = \vec{0}$$

This is **3 equations** (one per x, y, z component) in **3 unknowns** (`p`, `r`, `t`).

---

## Why Newton-Raphson?

`W(p, r, 0)` is a **nonlinear** function — it involves trigonometric functions (from superelevation rotations), arc-length parameterizations, and curved geometry. There is no closed-form algebraic solution. You can't just rearrange and isolate `p`, `r`, `t`.

### Why not something simpler?

| Alternative | Why it doesn't work here |
|---|---|
| **Analytical solution** | `W(p,r,0)` involves rotations, elevation profiles, arc-length integrals of arbitrary ground curves — no closed form exists. |
| **Grid search / brute force** | `p` is continuous over potentially long road segments. Sampling finely enough to find the intersection to `linear_tolerance` precision would require millions of evaluations per lane. Newton converges in ~5 iterations. |
| **Bisection** | Only works for 1D scalar functions. We have 3 coupled equations. |
| **Gradient descent** | Solves minimization (min ‖F‖²), not root-finding. Slower convergence (linear vs. quadratic). Can get stuck at local minima where ‖F‖ > 0 (near-miss, not an intersection). |
| **Fixed-point iteration** | Requires reformulating as x = g(x) with a contractive g. Hard to construct for this geometry and converges only linearly. |

---

## What is Newton-Raphson?

Newton-Raphson is an iterative root-finding method. Given `F(x) = 0` where `F` is nonlinear, it works by **repeatedly replacing the hard nonlinear problem with an easy linear one**:

1. **Start with a guess** `x₀ = (p₀, r₀, t₀)`.
2. **Linearize.** Near `x₀`, approximate `F` with its first-order Taylor expansion:
   `F(x₀ + Δx) ≈ F(x₀) + J(x₀) · Δx`, where `J` is the Jacobian.
3. **Solve the linear system.** Set the approximation to zero: `J · Δx = −F(x₀)`.
4. **Update.** `x₁ = x₀ + Δx`.
5. **Repeat** until `‖F(xₖ)‖` is small enough.

### Convergence properties

Newton-Raphson has **quadratic convergence** near a root: if the error at step `k` is `ε`, the error at step `k+1` is `~ Cε²`. Practically:

- If your initial guess is within ~1m of the true intersection, after 1 iteration you're within ~1mm, after 2 iterations within ~1μm.
- The code sets `kMaxIterations = 20`, but typically converges in 3–5 iterations.
- Convergence is checked by `‖F‖² < linear_tolerance²`.

### The one requirement: a good initial guess

Newton-Raphson can diverge or converge to a wrong root if the initial guess is too far away. That's why the code carefully seeds `(p₀, r₀, t₀)`:

1. **`p₀`**: from `GInverse` — the ground curve's own closest-point computation.
2. **`t₀`**: by projecting `W(p₀, 0, 0) − origin` onto the direction vector.
3. **`r₀ = 0`**: starting at the road reference line, which is usually close to any lane.

---

## What is the Jacobian and why is it needed?

The **Jacobian** `J` is the 3×3 matrix of partial derivatives of `F` with respect to each unknown:

$$J = \begin{bmatrix} \frac{\partial F}{\partial p} & \frac{\partial F}{\partial r} & \frac{\partial F}{\partial t} \end{bmatrix}$$

Since `F = W(p,r,0) − origin − t · direction`:

| Column | Partial derivative | What provides it |
|---|---|---|
| `∂F/∂p` | `∂W/∂p` — how the surface point moves as you slide along the road | `road_curve->WDot({p, r, 0})` |
| `∂F/∂r` | `∂W/∂r` — how the surface point moves as you slide laterally | `road_curve->RHat({p, r, 0})` |
| `∂F/∂t` | `−direction` — how the line point moves as you advance along the ray | just `−direction` |

The Jacobian is needed because:

1. **It encodes the local geometry.** Column 0 (tangent to the road) and column 1 (lateral direction) describe the road surface's orientation at the current guess. Column 2 is the line direction. Together they form a local coordinate frame.
2. **It makes the problem linear locally.** Near a solution, `F` is approximately `J · Δx`, so solving `J · Δx = −F` gives the exact correction for a linear surface — and a very good correction for a mildly curved one.
3. **It detects degeneracy.** If `det(J) ≈ 0`, the line is (nearly) tangent or parallel to the surface at that point — no clean intersection exists. The code checks this and skips the lane.

### Cramer's Rule

The 3×3 linear system `J · Δ = −F` is solved via Cramer's rule — a closed-form solution (no matrix library needed):

$$\Delta p = \frac{\det[-F \mid \text{col1} \mid \text{col2}]}{\det[J]}, \quad \Delta r = \frac{\det[\text{col0} \mid -F \mid \text{col2}]}{\det[J]}, \quad \Delta t = \frac{\det[\text{col0} \mid \text{col1} \mid -F]}{\det[J]}$$

This is what the code computes explicitly with scalar triple-product expansions.

---

## Contrast with `DoFindRoadPositionsAtXY`

In the vertical-line case, `direction = (0, 0, 1)`, so the z-equation decouples: only x and y matter, reducing to a 2D problem. That method uses an alternating Newton trick (solve `p` in 1D, then compute `r` by projection). With an arbitrary direction, all three components are coupled, so we need the full 3×3 system.

| Aspect | `DoFindRoadPositionsAtXY` | `FindRoadPositionsIntersectingLine` |
|---|---|---|
| Problem | 2 eqs / 2 unknowns (p, r) | 3 eqs / 3 unknowns (p, r, t) |
| Solver | Alternating 1D Newton (p) + r-projection | Full 3×3 Newton via Cramer's rule |
| Jacobian cols | `WDot` (∂W/∂p) only | `WDot` (∂W/∂p), `RHat` (∂W/∂r), −direction |
| Distance | 2D planar | 3D Euclidean |
| Singularity | Not possible (vertical line) | Detected via det(J) ≈ 0 (line tangent/parallel to surface), lane is skipped |
| Initialization | `GInverse(x,y)` | `GInverse` at XY of origin, then refined by projecting along the line |

---

## Toy Example — Step by Step

### Setup

**Road surface** (a parabolic ramp — elevation grows quadratically):

$$W(p, r, 0) = (p,\; r,\; 0.1 p^2)$$

**Line** (a ray shooting down into the ramp):

$$\text{origin} = (0, 0, 3), \quad \text{direction} = (1, 0.5, -1)$$
$$\text{point}(t) = (t,\; 0.5t,\; 3 - t)$$

### The equations

$$F(p, r, t) = W(p, r, 0) - \text{origin} - t \cdot \text{direction}$$

Expanding component by component — `W(p,r,0)` gives us the 3D point `(p, r, 0.1p²)` on the road, the line point is `(0 + t·1, 0 + t·0.5, 3 + t·(−1))`:

$$F = \underbrace{\begin{pmatrix} p \\ r \\ 0.1p^2 \end{pmatrix}}_{W(p,r,0)} - \underbrace{\begin{pmatrix} 0 \\ 0 \\ 3 \end{pmatrix}}_{\text{origin}} - t \cdot \underbrace{\begin{pmatrix} 1 \\ 0.5 \\ -1 \end{pmatrix}}_{\text{direction}} = \begin{pmatrix} p - t \\ r - 0.5t \\ 0.1p^2 + t - 3 \end{pmatrix}$$

The Jacobian:

$$J = \begin{bmatrix} 1 & 0 & -1 \\ 0 & 1 & -0.5 \\ 0.2p & 0 & 1 \end{bmatrix}$$

Notice: the Jacobian **depends on `p`** (because of the `0.2p` term from differentiating `0.1p²`) — it changes at each iteration. This is why we must recompute it every step.

---

### Iteration 1

**Current guess:** `p = 0, r = 0, t = 0`

**① Evaluate F:**

$$F(0, 0, 0) = \begin{pmatrix} 0 \\ 0 \\ -3 \end{pmatrix}, \quad \|F\| = 3$$

Not zero — we haven't converged.

**② Build the Jacobian at `p = 0`:**

$$J = \begin{bmatrix} 1 & 0 & -1 \\ 0 & 1 & -0.5 \\ 0 & 0 & 1 \end{bmatrix}$$

**③ Solve `J · Δ = −F` using Cramer's rule.** Right-hand side: `−F = (0, 0, 3)`.

`det(J) = 1·(1·1 − (−0.5)·0) − 0·(…) + (−1)·(0·0 − 1·0) = 1`

For `Δp` — replace column 0 with `(0, 0, 3)`:

$$\det \begin{bmatrix} 0 & 0 & -1 \\ 0 & 1 & -0.5 \\ 3 & 0 & 1 \end{bmatrix} = 0 - 0 + (-1)(0 - 3) = 3 \quad \Rightarrow \quad \Delta p = 3/1 = 3$$

For `Δr` — replace column 1 with `(0, 0, 3)`:

$$\det \begin{bmatrix} 1 & 0 & -1 \\ 0 & 0 & -0.5 \\ 0 & 3 & 1 \end{bmatrix} = 1 \cdot (0 + 1.5) = 1.5 \quad \Rightarrow \quad \Delta r = 1.5/1 = 1.5$$

For `Δt` — replace column 2 with `(0, 0, 3)`:

$$\det \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 3 \end{bmatrix} = 3 \quad \Rightarrow \quad \Delta t = 3/1 = 3$$

**④ Update:**

$$p = 0 + 3 = 3, \quad r = 0 + 1.5 = 1.5, \quad t = 0 + 3 = 3$$

---

### Iteration 2

**① Evaluate F at `(3, 1.5, 3)`:**

$$F(3, 1.5, 3) = \begin{pmatrix} 3 - 3 \\ 1.5 - 1.5 \\ 0.9 + 3 - 3 \end{pmatrix} = \begin{pmatrix} 0 \\ 0 \\ 0.9 \end{pmatrix}, \quad \|F\| = 0.9$$

Smaller but not zero. The z-component is off because the parabolic surface curves away from the tangent plane used in iteration 1.

**② Build the Jacobian at `p = 3`:**

$$J = \begin{bmatrix} 1 & 0 & -1 \\ 0 & 1 & -0.5 \\ 0.6 & 0 & 1 \end{bmatrix}$$

**③ Solve `J · Δ = −F = (0, 0, −0.9)` via Cramer's rule.**

`det(J) = 1·(1 − 0) − 0 + (−1)·(0 − 0.6) = 1 + 0.6 = 1.6`

`Δp`: replace column 0:

$$\det \begin{bmatrix} 0 & 0 & -1 \\ 0 & 1 & -0.5 \\ -0.9 & 0 & 1 \end{bmatrix} = -0.9 \quad \Rightarrow \quad \Delta p = -0.9/1.6 = -0.5625$$

`Δr`: replace column 1:

$$\det \begin{bmatrix} 1 & 0 & -1 \\ 0 & 0 & -0.5 \\ 0.6 & -0.9 & 1 \end{bmatrix} = -0.45 \quad \Rightarrow \quad \Delta r = -0.45/1.6 = -0.28125$$

`Δt`: replace column 2:

$$\det \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0.6 & 0 & -0.9 \end{bmatrix} = -0.9 \quad \Rightarrow \quad \Delta t = -0.9/1.6 = -0.5625$$

**④ Update:**

$$p = 3 - 0.5625 = 2.4375, \quad r = 1.5 - 0.28125 = 1.21875, \quad t = 3 - 0.5625 = 2.4375$$

---

### Iteration 3 (quick check)

$$F(2.4375, 1.21875, 2.4375) = \begin{pmatrix} 0 \\ 0 \\ 0.0316 \end{pmatrix}, \quad \|F\| = 0.0316$$

Much smaller. One more iteration would bring it below `10⁻⁴`, another below `10⁻⁹` (quadratic convergence: error gets **squared** each step).

---

## Summary: The Workflow

```
REPEAT:
  ① Evaluate F(p, r, t)                →  "How far off am I?"
  ② If ‖F‖ < tolerance → STOP           →  "Close enough — converged!"
  ③ Build Jacobian J at (p, r, t)       →  "How does the surface + line behave locally?"
  ④ Solve J · Δ = −F  (Cramer's rule)   →  "Which way and how far should I step?"
  ⑤ Update: p += Δp,  r += Δr,  t += Δt
```

Cramer's rule is the **method used inside step ④** to solve the 3×3 linear system.
Newton-Raphson is the **overall loop** (steps ①–⑤).
They work together: Newton sets up the linear system, Cramer solves it.
