// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @file road_curve_design.h
/// @page malidrive_road_curve_design Malidrive RoadCurve Design
/// @author Agustin Alba Chicar
/// @author Steven Peters
/// @date July 29, 2020
/// @tableofcontents
///
/// @section road_curve_concepts RoadCurve concepts and definitions
///
/// RoadCurve defines an interface for a path in a Segment object surface.
/// The path is defined by an elevation and superelevation Function objects and
/// a GroundCurve reference curve. The latter is a C¹ function in the
/// @f$ z=0 @f$ plane. Its domain is constrained in
/// @f$ [GroundCurve::p0(); GroundCurve::p1()] @f$ interval and it should map a
/// ℝ² curve.
///
/// As per notation, @f$ p @f$ is the parameter of the reference curve, not
/// necessarily arc length @f$ s @f$, and function interpolations and function
/// derivatives as well as headings and heading derivatives are expressed in
/// INERTIAL Frame coordinates.
///
/// The geometry here revolves around an abstract "world function":
///
/// @f$ W: (p,r,h) \mapsto (x,y,z) ∈ ℝ³ @f$
///
/// which maps a `Lane`-frame position to its corresponding representation in
/// world coordinates (with the caveat that instead of the lane's native
/// longitudinal coordinate `s`, the reference curve parameter `p` is used).
///
/// @f$ W @f$ is derived from three functions which define the lane:
///
/// - @f$ G: p \mapsto (x,y) @f$ : the reference ground curve (a `GroundCurve` implementation).
/// - @f$ Z: p \mapsto z @f$ : the elevation function (a `Function` implementation).
/// - @f$ Θ: p \mapsto θ @f$ : the superelevation function (a `Function` implementation).
///
/// as:
///
/// @f$ (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_{αβγ} x (0, r, h) @f$
///
/// where:
///
/// - @f$ R_{αβγ} @f$ is the roll/pitch/yaw rotation given by angles:
///
///   - @f$ α = Θ(p) @f$
///   - @f$ β = -atan2(\frac{dZ}{dp}, sqrt((\frac{dG_x}{dp})^2 + (\frac{dG_y}{dp})^2)) @f$ at @f$ p @f$
///   - @f$ γ = atan2(\frac{dG_y}{dp}, \frac{dG_x}{dp}) @f$ at @f$ p @f$
///
/// (@f$ R_{αβγ} @f$ is essentially the orientation of the @f$ (s,r,h) @f$ `Lane`-frame
/// at a location @f$ (s,0,0) @f$ on the reference-line of the lane.  However, it
/// is *not* necessarily the correct orientation at @f$ r \ne 0 @f$ or @f$ h \ne 0 @f$.)
///
/// The @f$ W(p,r,h) @f$ "world function" is defined by the RoadCurve referenced by a
/// Lane's Segment. A Lane is also defined by a @f$ r0 @f$ lateral offset with respect
/// to the reference curve of the RoadCurve. Thus, a mapping from the local
/// @f$ (s,r,h) @f$ lane-frame of the Lane becomes:
///
/// @f$ (x,y,z) = L(s,r,h) = W(P(s, r0), r + r0, h) @f$
///
/// Where
/// - @f$ P: (s, r0) \mapsto (p) @f$ is a (potentially non-linear) function dependent on
///   the RoadCurve's reference-curve, elevation, and superelevation functions.
/// - @f$ L: (s, r, h) \mapsto (x, y, z) @f$ is the "lane function", a lateral offset of the
///   world function which lays on the road surface. Note that it is
///   parametrized with its arc length @f$ s @f$ instead of an arbitrary
///   parameter @f$ p @f$.
///
/// @subsection mapping_the_inertial_frame Mapping the INERTIAL Frame
///
/// Let @f$ p_L = (p, r, h) @f$ be a point in the RoadCurve surface. The image
/// of the world function @f$ W(p_L) @f$ can be expressed as:
///
/// @f$ W(p_L) = W(p, r, h) = (G(p), Z(p)) + R_{αβγ} x (0, r, h) @f$
///
/// @subsection tangent_vector_road_surface Computing the tangent vector
///
/// Let @f$ p_L = (p, r, h) @f$ be a point in the RoadCurve surface. The tangent
/// vector expressed as the derivative of the world function with respect to
/// @f$ p @f$ can be expressed as:
///
/// @f$ \frac{\partial W(p_L)}{\partial p} = \frac{\partial W(p, r, h)}{\partial p} @f$
///
/// @f$ \frac{\partial W(p_L)}{\partial p} = (\frac{dG(p)}{dp}, \frac{dZ(p)}{dp}) + \frac{dR_{αβγ}}{dp} x (0, r, h) @f$
///
/// and
///
/// @f$ \frac{dR_{αβγ}}{dp} = (\frac{dR_{αβγ}}{dα} \frac{dR_{αβγ}}{dβ} \frac{dR_{αβγ}}{dγ}) x (\frac{dα}{dp},
/// \frac{dβ}{dp}, \frac{dγ}{dp}) @f$
///
/// There are analytic definitions for each derivative expressed above via the
/// API of `GroundCurve`, `Function`, and `RollPitchYaw` classes.
///
/// @subsection orientation Orientation
///
/// @subsubsection orientation_at_the_centerline Orientation at the centerline
///
/// Specifically, the orientation at the centerline is the orientation of the
/// surface in the INERTIAL frame at any point mapped by the world function
/// whose domain is @f$ (p, 0, 0) @f$ (which yields a curve on the surface).
///
/// - @f$ α = Θ(p) @f$
/// - @f$ β = -atan2(\frac{dZ}{dp}, \sqrt{(\frac{dG_x}{dp})^2 + (\frac{dG_y}{dp})^2}) @f$
/// - @f$ γ = atan2(\frac{dG_y}{dp}, \frac{dG_x}{dp}) @f$
///
/// Then, @f$ R_{αβγ} = R(α, β, γ) @f$
///
/// @subsubsection orientation_at_any_point Orientation at any point in the road volume
///
/// Lanes in `maliput` define a volume as a orthonormal extrusion of the surface
/// in the elevation bounds. The orientation @f$ p_L = (p, r, h) @f$ is computed
/// as:
///
/// @f$ \hat{s} = ||\frac{\partial W(p, r, h)}{\partial p}|| @f$
///
/// @f$ \hat{r} = R_{αβγ} * (0., 1., 0.) @f$
///
/// > Note that in certain cases @f$ \hat{s}, \hat{r} @f$ are not orthogonal, as documented in
/// > https://github.com/ToyotaResearchInstitute/malidrive/issues/458#issuecomment-663767176
///
/// And,
///
/// - @f$ γ = atan2(\hat{s}[1], \hat{s}[0]) @f$
/// - @f$ β = atan2(-\hat{s}[2], ||(\hat{s}[0], \hat{s}[1])||) @f$
/// - @f$ α = atan2(\frac{\hat{r}[2]}{cos(β)}, \frac{\hat{r}[1] x \hat{s}[0] - \hat{r}[0] x \hat{s}[1])}{cos(β)}) @f$
///
/// Then, we can compute the rotation matrix out of @f$ (α, β, γ) @f$.
///
/// @subsection inverse_function Inverse function
///
/// Given a point @f$ p_W = (x, y, z) @f$ there might exist a point
/// @f$ p_L = (p, r, h)@f$ that satisfies @f$ W(p_L) = p_W @f$.
///
/// The inverse function is procedurally computed by:
///
/// - Compute the @f$ p @f$, the parameter value that satisfies
///   @f$ p_{L_0} = (x, y) @f$ such that @f$ G(p) = p_{L_0} @f$ (the inverse
///   function of G, which has a closed analytic solution).
/// - Iterate while not satisfying tolerance (i.e. Δp > linear_tolerance):
///   - Compute @f$ p_{W_0} @f$, the image of @f$ W(p, 0, 0)@f$.
///   - Compute @f$ Δp_W = p_W - p_{W_0} @f$
///   - Compute @f$ W^{'}(p_{W_0}) = \frac{\partial W(p, 0, 0)}{\partial p}@f$
///   - Compute Δp as first order Newton approximation: @f$ Δp = \frac{<Δp_W, W^{'}(p_{W_0})>}{|| W^{'}(p_{W_0}) ||} @f$
///   - Compute @f$ p = p + Δp @f$.
/// - Compute @f$ \hat{s}, \hat{r}, \hat{h} = \hat{s} x \hat{r} @f$.
/// - Compute @f$ p_L = (p, <\hat{r}, Δp_W>, <\hat{h}, Δp_W>)@f$.
///
/// @section math_reformulation Alternate formulation of math problem
///
/// Consider a parameterized curve in 3D space called @f$ C_{0} @f$ whose coordinates in an inertial frame
/// with orthonormal basis vectors @f$ \hat{x}, \hat{y}, \hat{z} @f$ are defined by
/// continuous functions of @f$ p ∈ [p_0, p_1] @f$:
/// @f$ x(p), y(p), z(p) @f$ with the requirement that @f$ x(p), y(p) @f$
/// do not jointly stagnate with respect to @f$ p @f$
/// (i.e. that there exists a value of @f$ k @f$ such that
/// @f$ (\frac{dx}{dp})^2 + (\frac{dy}{dp})^2 >= k > 0 @f$).
///
/// The unit vector tangent to @f$ C_{0} @f$ is given by
/// @f$ \hat{t}(p) = \frac{1}{\sqrt{(\frac{dx}{dp})^2 + (\frac{dy}{dp})^2 + (\frac{dz}{dp})^2}}
/// [ \frac{dx}{dp}, \frac{dy}{dp}, \frac{dz}{dp} ]^T = \frac{C_{0}'}{||C_{0}'||}@f$
///
/// A vector @f$ n_0(p) @f$ is defined as the component of @f$ \hat{z} @f$ that is orthogonal
/// to @f$ \hat{t} @f$ as @f$ n_0(p) = \hat{z} - <\hat{t}, \hat{z}> \hat{t} @f$.
/// The vector @f$ n_0(p) @f$ has non-zero length if @f$ \hat{z} @f$ and @f$ \hat{t} @f$
/// are not identical, and the condition restricting joint stagnation of @f$ x(p), y(p) @f$
/// is sufficient to guarantee this.
/// A unit vector @f$ \hat{n}_0 @f$ is defined by normalizing @f$ n_0 @f$:
/// @f$ \hat{n}_0(p) = \frac{n_0}{||n_0||} @f$
///
/// A third unit vector orthogonal to both @f$ \hat{n}_0(p), \hat{t}(p) @f$ is defined as
/// @f$ \hat{b}_0(p) = \hat{n}_0(p) \times \hat{t}(p) @f$. The unit vectors
/// @f$ \hat{t}(p), \hat{b}_0(p), \hat{n}_0(p) @f$ form an orthonormal basis.
///
/// Another orthonormal basis is defined by rotating @f$ \hat{t}(p), \hat{b}_0(p), \hat{n}_0(p) @f$
/// about @f$ \hat{t}(p) @f$ by an angle specified by a continuous function @f$ Θ(p) @f$.
/// Using the [Rodrigues rotation formula](https://math.stackexchange.com/a/142831):
/// with components of @f$ \hat{t} @f$ in the inertial frame given as
/// @f$ \hat{t}(p) = t_x \hat{x} + t_y \hat{y} + t_z \hat{z} @f$,
/// identity matrix @f$ I @f$,
/// and @f$ W = \begin{pmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{pmatrix} @f$,
/// a Rotation matrix is defined as @f$ R_Θ(p) = I + \sin Θ(p) W + 2 \sin^2 \frac{Θ(p)}{2} W^2 @f$.
/// By construction, rotation by @f$ R_Θ(p) @f$ does not affect @f$ \hat{t}(p) @f$,
/// so the new basis vectors are defined as @f$ \hat{t}, \hat{b} = R_Θ(p) \hat{b}_0, \hat{n} = R_Θ(p) \hat{n}_0 @f$.
/// A rotation matrix @f$ R_{\hat{t},\hat{b},\hat{n}} @f$ is defined with these unit vectors as its columns:
/// @f$ R_{\hat{t},\hat{b},\hat{n}} = [\hat{t},\hat{b},\hat{n}] @f$.
///
/// Note that these basis vectors @f$ \hat{t}, \hat{b}, \hat{n} @f$ bear some resemblance to the basis vectors
/// of a Frenet-Serret frame, since the tangent vector @f$ \hat{t} @f$ is identical in both formulations,
/// but the other unit vectors have significant differences. Whereas the Frenet-Serret normal unit vector is defined
/// based on the curvature of the 3D curve, in this formulation it is constrained to be a projection of the
/// inertial @f$ \hat{z} @f$ axis subject to a specified rotation. Additionally, the order of the unit vectors differs,
/// with Frenet-Serret defining a right-hand basis with tangent, normal, and binormal
/// (@f$ \hat{T} \times \hat{N} = \hat{B} @f$), while this formulation
/// defines the right-hand basis with tangent, binormal, normal (@f$ \hat{t} \times \hat{b} = \hat{n} @f$).
///
/// To compute the derivative of the rotation matrix @f$ R_{\hat{t},\hat{b},\hat{n}} @f$ with respect to @f$ p @f$,
/// the following relationship is used that involves @f$ ω_{\hat{t},\hat{b},\hat{n}} @f$, which is like an angular
/// velocity of the basis vectors expressed in the inertial frame with respect to changes in @f$ p @f$ rather than
/// changes in time:
/// @f$ \frac{dR_{\hat{t},\hat{b},\hat{n}}}{dp} = ω_{\hat{t},\hat{b},\hat{n}} \times R_{\hat{t},\hat{b},\hat{n}} @f$.
/// The term @f$ ω_{\hat{t},\hat{b},\hat{n}} @f$ is a a sum of two components:
/// a component @f$ ω_{⟂ \hat{t}} @f$ that is orthogonal to @f$ \hat{t} @f$ and a component parallel to
/// @f$ \hat{t} @f$, which is derived from the derivative of the rotation function @f$ Θ'(p) = \frac{dΘ}{dp} @f$.
///
/// @f$ ω_{\hat{t},\hat{b},\hat{n}} = ω_{⟂ \hat{t}} + Θ'(p) \hat{t} @f$
///
/// Since @f$ \hat{t} @f$ has unit length, its derivative @f$ \frac{d\hat{t}}{dp} @f$ can be expressed as a cross
/// product with the angular velocity component @f$ ω_{⟂ \hat{t}} @f$ that is orthogonal to @f$ \hat{t} @f$:
///
/// @f$ \frac{d\hat{t}}{dp} = ω_{⟂ \hat{t}} \times \hat{t} @f$.
///
/// Recalling that @f$ \hat{t} = \frac{C_{0}'}{||C_{0}'||}@f$, its derivative is computed as:
///
/// @f$ \frac{d\hat{t}}{dp} = \frac{||C_{0}'|| C_{0}'' - \frac{d||C_{0}'||}{dp} C_{0}'}{||C_{0}'||^2} @f$
///
/// Recalling that @f$ ||C_{0}'|| = \sqrt{x'^2 + y'^2 + z'^2} @f$,
///
/// it can be differentiated as:
///
/// @f$ \frac{d||C_{0}'||}{dp} = \frac{1}{2 ||C_{0}'||} (2 x' x'' + 2 y' y'' + 2 z' z'') @f$.
///
/// @f$ \frac{d||C_{0}'||}{dp} = \frac{<C_{0}', C_{0}''>}{||C_{0}'||} @f$
///
/// @f$ \frac{d||C_{0}'||}{dp} = <\hat{t}, C_{0}''> @f$
///
/// and substituted back as
///
/// @f$ \frac{d\hat{t}}{dp} = \frac{||C_{0}'|| C_{0}'' - <\hat{t}, C_{0}''> C_{0}'}{||C_{0}'||^2} @f$
///
/// @f$ \frac{d\hat{t}}{dp} = \frac{C_{0}'' - <\hat{t}, C_{0}''> \hat{t}}{||C_{0}'||} @f$
///
/// @f$ \frac{d\hat{t}}{dp} = \frac{C_{0 ⟂ \hat{t}}''}{||C_{0}'||} @f$
///
/// where @f$ C_{0 ⟂ \hat{t}}'' @f$ is the component of @f$ C_{0}'' @f$ that is orthogonal to @f$ \hat{t} @f$.
///
/// Then
///
/// @f$ \frac{d\hat{t}}{dp} = \frac{C_{0 ⟂ \hat{t}}''}{||C_{0}'||} = ω_{⟂ \hat{t}} \times \hat{t} @f$.
///
/// Since both @f$ C_{0 ⟂ \hat{t}}'' @f$ and @f$ ω_{⟂ \hat{t}} @f$ are orthogonal to @f$ \hat{t} @f$,
/// the following can be shown to be equivalent:
///
/// @f$ ω_{⟂ \hat{t}} \times \hat{t} = \frac{C_{0 ⟂ \hat{t}}''}{||C_{0}'||} @f$
///
/// and
///
/// @f$ ω_{⟂ \hat{t}} = \hat{t} \times \frac{C_{0 ⟂ \hat{t}}''}{||C_{0}'||} @f$.
///
/// The derivative of the rotation matrix is thus
///
/// @f$ \frac{dR_{\hat{t},\hat{b},\hat{n}}}{dp} =
///   (\hat{t} \times \frac{C_{0 ⟂ \hat{t}}''}{||C_{0}'||} + Θ'(p) \hat{t}) \times R_{\hat{t},\hat{b},\hat{n}} @f$.
///
/// Consider a surface that contains @f$ C_{0} @f$ and whose surface normal at each point on
/// @f$ C_{0} @f$ is given by @f$ \hat{n} @f$.
///
/// @subsection at_a_lateral_offset Rotation at a lateral offset
///
/// Let the previous parametrized curve in 3D be called @f$ C_{0} @f$ and let
/// @f$ C_{offset} @f$ be an offset curve at a distance @f$ r_{(p)} @f$ along @f$ \hat{b}_{(p, 0)} @f$ on
/// the surface where @f$ C_{0} @f$ is defined. @f$ r_{(p)} @f$ is a C¹ function
/// of @f$ p @f$. Then, the curve @f$ C_{offset_{(p)}} @f$ can be expressed as:
///
/// @f$ C_{offset_{(p)}} = C_{0_{(p)}} + r_{(p)} \hat{b}_{(p, 0)}
/// @f$
///
/// We would like to compute the rotation matrix out of the frame basis for any
/// @f$ p ∈ [p_0, p_1] @f$ in @f$ C_{offset_{(p)}} @f$. To that end, we compute
/// @f$ C_{offset_{(p)}} @f$ derivative with respect to @f$ p @f$:
///
/// @f$ \frac{dC_{offset_{(p)}}}{dp} = \frac{dC_{0_{(p)}}}{dp} + \frac{dr_{(p)}}{dp} \hat{b}_{(p, 0)}
/// + r_{(p)} \frac{d\hat{b}_{(p, 0)}}{dp} @f$
///
/// And to compute @f$ \hat{t}_{(p,r)}, \hat{b}_{(p,r)}, \hat{n}_{(p,r)} @f$
/// (i.e. @f$ \hat{t}_{offset}, \hat{b}_{offset}, \hat{n}_{offset} @f$ ) we use
/// the same procedure as above.
///
/// @subsection motion_derivatives Motion derivatives
///
/// Let the previous parametrized curve in 3D be called @f$ C_{0} @f$ and let
/// @f$ C_{offset} @f$ be an offset curve at a distance @f$ r_{(p)} @f$ along  @f$ \hat{b}_{(p, 0)} @f$ on
/// the surface where @f$ C_{0} @f$ is defined. And let @f$ r_{k} @f$ be a scalar
/// measured in the @f$ \hat{b}_{(p, r_{(p)})} @f$ axis from a point in the
/// @f$ C_{offset} @f$ curve at a certain @f$ p @f$ value. In addition,
/// let @f$ v @f$ be a linear speed vector expressed in the orthonormal basis
/// @f$ \hat{t}_{offset}, \hat{b}_{offset}, \hat{n}_{offset} @f$ at
/// @f$ C_{offset_{(p)}} @f$.
///
/// We are interested in measuring the derivative at the @f$ r_{k} @f$ distance
/// from @f$ C_{offset(p)} @f$ along @f$ \hat{b}_{(p, 0)} @f$. That point in the INERTIAL Frame can be
/// expressed as:
///
/// @f$ C_{offset_{(p, r_{k})}} = C_{offset_{(p)}} + r_{k} \hat{b}_{p, r_{(p)}} @f$
///
/// @f$ C'_{offset_{(p, r_{k})}} = C'_{offset_{(p)}} + r_{k} \hat{b}'_{p, r_{(p)}} @f$
///
/// > TODO(agalbachicar / scpeters): I think the next step should be something like the following:
/// > @f$ C'_{offset_{(p, r_{k})}} = C'_{offset_{(p)}} + r_{k} ω_{⟂ \hat{b}_{p, r_{(p)}}} \times \hat{b}_{p, r_{(p)}}
/// @f$
///
/// @f$ C'_{offset_{(p, r_{k})}} = (1 - r_{k} κ_{offset_{(p)}}) C'_{offset_{(p)}} @f$
///
/// Where @f$ κ_{offset_{(p)}} = || \hat{t}'_{(p, r_{(p)})} || @f$. Consequently,
/// it is expected a change @f$ v @f$ by a factor of
/// @f$ (1 - r_{k} κ_{offset_{(p)}}) @f$ in the @f$ \hat{t}_{(p, r_{(p)})} @f$
/// direction.
///
/// > TODO(agalbachicar / scpeters): Given an offset in the
/// > @f$ \hat{n}_{p, r_{(p)}}@f$ direction at @f$ C_{offset_{(p)}} @f$, is there any
/// > *vertical* curvature like quantity that we can introduce to scale
/// > @f$ v @f$ ?
///
/// Another route to derive the same, i.e. the scaling of @f$ v @f$ at a certain
/// position in the curve offset (@f$ C_{offset} @f$) frame at @f$ p @f$, would imply the
/// following procedure:
///
/// @f$ C_{offset_{(p)}} = C_{0(p)} + R_{\hat{t}, \hat{b}, \hat{n}} \times [0, r_{(p)}, 0]^T @f$
///
/// And the position of a point with a displacement @f$ [0, r_k, h_k] @f$ from
/// the curve offset at @f$ p @f$ is:
///
/// @f$ C_{offset_{(p, r_k, h_k)}} = C_{0(p)} + R_{\hat{t}, \hat{b}, \hat{n}} \times [0, r_{(p)}, 0]^T +
/// R_{offset_{\hat{t}, \hat{b}, \hat{n}}} \times [0, r_k, h_k]^T @f$
///
/// @f$ C_{offset_{(p, r_k, h_k)}} = C_{offset_{(p)}} + R_{offset_{\hat{t}, \hat{b}, \hat{n}}} \times [0, r_k, h_k]^T
/// @f$
///
/// Where @f$ R_{offset_{\hat{t}, \hat{b}, \hat{n}}} @f$ is the rotation matrix at
/// @f$ C_{offset_{(p)}} @f$ which is computed from @f$ \hat{t}_{offset}, \hat{b}_{offset}, \hat{n}_{offset}@f$.
/// The procedure to compute the basis is exactly the same as it was computed
/// before to derive @f$ R_{\hat{t}, \hat{b}, \hat{n}} @f$ but, this time,
/// @f$ C'_{offset_{(p)}} @f$ is required to obtain @f$ \hat{t}_{offset} @f$ and
/// the other vectors.
///
/// To compute @f$ C'_{offset_{(p, r_k, h_k)}} @f$, we make use of @f$ C'_{offset_{(p)}} @f$:
///
/// @f$ C'_{offset_{(p, r_k, h_k)}} = C'_{offset_{(p)}} + R'_{offset_{\hat{t}, \hat{b}, \hat{n}}} \times [0, r_k, h_k]^T
/// @f$
///
/// We can apply an analogous procedure to compute @f$ R'_{offset_{\hat{t}, \hat{b}, \hat{n}}} @f$ and
/// replace the subscript @f$ _{0} @f$ by @f$ _{offset} @f$. However, the
/// expansion of @f$ C''_{offset} @f$ requires @f$ R''_{\hat{t}, \hat{b}, \hat{n}} @f$
/// to exist, leading to the need of @f$ C'''_{0} @f$ to exist too.
///
/// > TODO(agalbachicar / scpeters): Determine whether @f$ C'''_{0} @f$ is
/// > really needed or not and if for the geometries we are working with there
/// > is a real need of it to be other than a zero, or a well-known constant.
///
/// For planar curves, it is expected a change @f$ v @f$ by a factor of
/// @f$ \frac{|| C'_{offset_{(p, r_k, h_k)}} ||}{|| C'_{offset_{(p)}} ||} @f$ which is the scale ratio
/// of tangent vectors.
///
/// @section questions Questions
///
/// @subsection comparing_definitions_of_frenet_frame Comparing definitions of the Frenet-like frame
///
/// This document contains two separate approaches for defining a Frenet-like coordinate frame for each point
/// on a 3D curve @f$ C_0 @f$: using `RollPitchYaw` and by computing unit vectors directly.
///
/// **Q: Are these approaches equivalent? Are they correct?**
///
/// @subsection roll_pitch_yaw_orthogonality `RollPitchYaw` orthogonality
///
/// Consider a point fixed to @f$ C_0 @f$'s Frenet-like frame.
/// By computing the velocity of that point, an additional (recursive) Frenet-like frame can be computed at
/// that point, starting with a tangent vector computed from the velocity of that point.
///
/// **Q: For the formulation based on `RollPitchYaw`, the tangent and binormal unit vectors of that recursive
/// Frenet-like frame are not orthogonal. Where is the problem? Should the approach based on unit vectors
/// be preferred?**
///
/// @subsection continuity_requirements Continuity requirements
///
/// In the road context, our documentation says in several places that C¹ continuity is required for the
/// ground curve function, but the formulation defined using unit vectors includes references to the
/// second derivative of @f$ C_0 @f$ when computing the velocity of points fixed to the Frenet-like frame.
/// Furthermore, when defining an additional curve at a specified offset from @f$ C_0 @f$ and computing a
/// Frenet-like frame for the offset curve, then computing the velocity of points fixed to the offset
/// Frenet-like frame have references to the third derivative of @f$ C_0 @f$.
///
/// **Q: What continuity constraints are required for the curve** @f$ C_0 @f$ **? And to the surface
/// where the curve** @f$ C_0 @f$ **and its offsets are embedded?**
///
