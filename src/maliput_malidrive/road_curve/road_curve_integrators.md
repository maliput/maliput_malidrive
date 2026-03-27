# Road Model Construction and Integrators in Malidrive

## 1. How the Road Model is Constructed (`RoadCurve`)

The `RoadCurve` class represents the 3D geometry of a road segment. It is not defined by a single simple equation but is constructed by combining three distinct functions:

1.  **Ground Curve ($G(p)$)**: A 2D curve in the $xy$-plane (flat ground). This is the "spine" of the road. One common implementation is `ParamPoly3GroundCurve` (parametric cubic polynomial), but other `GroundCurve` subclasses exist (e.g., `ArcGroundCurve`, `LineGroundCurve`).
2.  **Elevation ($Z(p)$)**: A function that defines the height ($z$) of the road at any point $p$ along the curve.
3.  **Superelevation ($\Theta(p)$)**: A function that defines the banking angle (roll) of the road at any point $p$.

**The "World Function" $W(p, r, h)$**:
The code combines these three components into a single mapping called the World Function. It calculates the 3D world coordinates $(x, y, z)$ for any point defined by:
*   $p$: Longitudinal parameter along the road.
*   $r$: Lateral offset (distance to the left/right of the center).
*   $h$: Height above the road surface.

$$ W(p, r, h) = \underbrace{(G(p), Z(p))}_{\text{Reference Line}} + \underbrace{R_{\alpha\beta\gamma}(p)}_{\text{Orientation}} \cdot (0, r, h) $$

## 2. The Role of Integrators (`RoadCurveOffset`)

The core problem is that the parameter **$p$ is rarely the same as the distance traveled (arc length $s$)**.

*   For a line, $p=s$.
*   For a complex curve (like a cubic polynomial with elevation changes), moving $1.0$ unit in $p$ might move $1.2$ meters or $0.9$ meters in the real world depending on the curve's shape.

To perform physics simulations or drive a car along the road, we need to know exactly where we are in **meters ($s$)**, not in abstract parameters ($p$). Since there is no simple formula to convert $p$ to $s$ for these complex shapes, the system uses **Numerical Integrators**.

The `RoadCurveOffset` class manages this relationship. It is constructed with:
*   A `const RoadCurve*` — the road geometry to integrate over.
*   A `const Function* lane_offset` — a function $r(p)$ describing how the lateral offset of a lane varies with $p$ (not a simple scalar).
*   `p0`, `p1` — the parameter range.
*   `integrator_accuracy_multiplier` — a multiplier to tune the accuracy of the integrator (1.0 for most cases).

Internally it creates two specific integrators:

### A. Forward Integrator: $s(p)$
**"I am at parameter $p$. How many meters have I traveled?"**

This is calculated by integrating the "speed" of the curve function along the path.
*   **Math:** $s(p) = \int_{p_0}^{p} \| W'(t) \| dt$
*   **Code:** `maliput::drake::ArcLengthIntegrator`
*   **Implementation:** It sums up tiny steps along the curve to calculate the total length.

### B. Inverse Integrator: $p(s)$
**"I have traveled $s$ meters. What is my parameter $p$?"**

This is harder. It requires solving an Initial Value Problem (ODE).
*   **Math:** $\frac{dp}{ds} = \frac{1}{\| W'(p) \|}$
*   **Code:** `maliput::drake::InverseArcLengthIntegrator`
*   **Implementation:** It steps through the curve until the accumulated length equals the target $s$.

### Summary
*   **RoadCurve** defines the **shape** (Geometry).
*   **RoadCurveOffset** defines the **metrics** (Distance).
*   **Integrators** are the bridge that translates the mathematical definition of the curve ($p$) into real-world distances ($s$) that vehicles can drive on.

## 3. Impact of Discontinuities on Integrators

If the World Function $W(p, r, h)$ presents a discontinuity, it is catastrophic for the numerical integrators.

Here is exactly how it affects the system:

### The Mathematical Assumption
Numerical integrators (like the Runge-Kutta methods used here) rely on the assumption that the function being integrated is **smooth** (specifically, that it has continuous derivatives, or is $C^1$ continuous). They predict the next point based on the current slope and curvature.

If $W$ is discontinuous, its derivative $W'$ is undefined (infinite) or jumps instantly.

### Effect on Forward Integration ($s$ from $p$)
The forward integrator calculates $s = \int \|W'(p)\| dp$.

*   **If there is a Kink ($C^1$ discontinuity):** The derivative $\|W'(p)\|$ jumps instantly (e.g., from 1.0 to 0.5). The integrator monitors "error" by comparing a full step vs. two half steps. At a jump, this error estimate spikes. The integrator responds by **shrinking the step size** drastically to try to satisfy the error tolerance. It may reduce the step size until it hits machine epsilon, causing the software to hang or throw a "step size too small" error.
*   **If there is a Gap ($C^0$ discontinuity):** The derivative is effectively infinite (a Dirac delta). The integrator cannot calculate the area under an infinite spike.

### Effect on Inverse Integration ($p$ from $s$)
The inverse integrator solves the ODE $\frac{dp}{ds} = \frac{1}{\|W'(p)\|}$.

*   **Singularities:** If the road geometry "stops" or folds back on itself (derivative becomes zero), the term $\frac{1}{\|W'(p)\|}$ approaches infinity. The integrator will fail to solve the Initial Value Problem because the gradient is vertical.
*   **Loss of Bijectivity:** If the function is discontinuous, the mapping between $p$ and $s$ is no longer strictly one-to-one in a smooth sense. The solver may get stuck in a local minimum or fail to converge to the target $s$.

### How Malidrive Prevents This
Because this issue is so critical, `maliput_malidrive` enforces continuity constraints **before** the integrators are ever allowed to run.

The `RoadCurve` constructor enforces the following checks (see `road_curve.cc`):

1.  `ground_curve`, `elevation`, and `superelevation` must not be `nullptr`.
2.  `ground_curve` **must** be G¹ (always enforced).
3.  `elevation` and `superelevation` **must** be G¹ — but only when the constructor's `assert_contiguity` flag is `true`. Some maps disable this check to tolerate minor discontinuities in elevation/superelevation profiles.
4.  All three functions must share the same parameter range ($p_0$, $p_1$) within `linear_tolerance`.

If you were to force a discontinuous curve into the `RoadCurve` object, the `RoadCurveOffset` integrators would likely enter an infinite loop of reducing step sizes, or the application would crash with an integration exception.
