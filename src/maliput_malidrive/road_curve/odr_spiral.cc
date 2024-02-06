#include "maliput_malidrive/road_curve/odr_spiral.h"

#include <cmath>

namespace odr {
namespace {
/* ====== LOCAL VARIABLES ====== */

/* S(x) for small x */
static double sn[6] = {
    -2.99181919401019853726E3,
    7.08840045257738576863E5,
    -6.29741486205862506537E7,
    2.54890880573376359104E9,
    -4.42979518059697779103E10,
    3.18016297876567817986E11,
};
static double sd[6] = {
    /* 1.00000000000000000000E0,*/
    2.81376268889994315696E2,
    4.55847810806532581675E4,
    5.17343888770096400730E6,
    4.19320245898111231129E8,
    2.24411795645340920940E10,
    6.07366389490084639049E11,
};

/* C(x) for small x */
static double cn[6] = {
    -4.98843114573573548651E-8,
    9.50428062829859605134E-6,
    -6.45191435683965050962E-4,
    1.88843319396703850064E-2,
    -2.05525900955013891793E-1,
    9.99999999999999998822E-1,
};
static double cd[7] = {
    3.99982968972495980367E-12,
    9.15439215774657478799E-10,
    1.25001862479598821474E-7,
    1.22262789024179030997E-5,
    8.68029542941784300606E-4,
    4.12142090722199792936E-2,
    1.00000000000000000118E0,
};

/* Auxiliary function f(x) */
static double fn[10] = {
    4.21543555043677546506E-1,
    1.43407919780758885261E-1,
    1.15220955073585758835E-2,
    3.45017939782574027900E-4,
    4.63613749287867322088E-6,
    3.05568983790257605827E-8,
    1.02304514164907233465E-10,
    1.72010743268161828879E-13,
    1.34283276233062758925E-16,
    3.76329711269987889006E-20,
};
static double fd[10] = {
    /*  1.00000000000000000000E0,*/
    7.51586398353378947175E-1,
    1.16888925859191382142E-1,
    6.44051526508858611005E-3,
    1.55934409164153020873E-4,
    1.84627567348930545870E-6,
    1.12699224763999035261E-8,
    3.60140029589371370404E-11,
    5.88754533621578410010E-14,
    4.52001434074129701496E-17,
    1.25443237090011264384E-20,
};

/* Auxiliary function g(x) */
static double gn[11] = {
    5.04442073643383265887E-1,
    1.97102833525523411709E-1,
    1.87648584092575249293E-2,
    6.84079380915393090172E-4,
    1.15138826111884280931E-5,
    9.82852443688422223854E-8,
    4.45344415861750144738E-10,
    1.08268041139020870318E-12,
    1.37555460633261799868E-15,
    8.36354435630677421531E-19,
    1.86958710162783235106E-22,
};
static double gd[11] = {
    /*  1.00000000000000000000E0,*/
    1.47495759925128324529E0,
    3.37748989120019970451E-1,
    2.53603741420338795122E-2,
    8.14679107184306179049E-4,
    1.27545075667729118702E-5,
    1.04314589657571990585E-7,
    4.60680728146520428211E-10,
    1.10273215066240270757E-12,
    1.38796531259578871258E-15,
    8.39158816283118707363E-19,
    1.86958710162783236342E-22,
};

static double polevl(double x, double* coef, int n)
{
    double  ans;
    double* p = coef;
    int     i;

    ans = *p++;
    i = n;

    do
    {
        ans = ans * x + *p++;
    } while (--i);

    return ans;
}

static double p1evl(double x, double* coef, int n)
{
    double  ans;
    double* p = coef;
    int     i;

    ans = x + *p++;
    i = n - 1;

    do
    {
        ans = ans * x + *p++;
    } while (--i);

    return ans;
}

static void fresnel(double xxa, double* ssa, double* cca)
{
    double f, g, cc, ss, c, s, t, u;
    double x, x2;

    x = fabs(xxa);
    x2 = x * x;

    if (x2 < 2.5625)
    {
        t = x2 * x2;
        ss = x * x2 * polevl(t, sn, 5) / p1evl(t, sd, 6);
        cc = x * polevl(t, cn, 5) / polevl(t, cd, 6);
    }
    else if (x > 36974.0)
    {
        cc = 0.5;
        ss = 0.5;
    }
    else
    {
        x2 = x * x;
        t = M_PI * x2;
        u = 1.0 / (t * t);
        t = 1.0 / t;
        f = 1.0 - u * polevl(u, fn, 9) / p1evl(u, fd, 10);
        g = t * polevl(u, gn, 10) / p1evl(u, gd, 11);

        t = M_PI * 0.5 * x2;
        c = cos(t);
        s = sin(t);
        t = M_PI * x;
        cc = 0.5 + (f * s - g * c) / t;
        ss = 0.5 - (f * c + g * s) / t;
    }

    if (xxa < 0.0)
    {
        cc = -cc;
        ss = -ss;
    }

    *cca = cc;
    *ssa = ss;
}

/**
 * compute the actual "standard" spiral, starting with curvature 0
 * @param s      run-length along spiral
 * @param cDot   first derivative of curvature [1/m2]
 * @param x      resulting x-coordinate in spirals local co-ordinate system [m]
 * @param y      resulting y-coordinate in spirals local co-ordinate system [m]
 * @param t      tangent direction at s [rad]
 */

void odrSpiral(double s, double cDot, double* x, double* y, double* t)
{
    double a;

    a = 1.0 / sqrt(fabs(cDot));
    a *= sqrt(M_PI);

    fresnel(s / a, y, x);

    *x *= a;
    *y *= a;

    if (cDot < 0.0)
        *y *= -1.0;

    *t = s * s * cDot * 0.5;
}

} // namespace

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end):
  s0_(s0), x0_(x0), y0_(y0), hdg0_(hdg0), length_(length), curv_start_(curv_start), curv_end_(curv_end)
{
    c_dot_ = (curv_end_ - curv_start_) / length_;
    s_start_ = curv_start_ / c_dot_;
    s_end_ = curv_end_ / c_dot_;
    s0_spiral_ = curv_start_ / c_dot_;
    odrSpiral(s0_spiral_, c_dot_, &x0_spiral_, &y0_spiral_, &a0_spiral_);
}

maliput::math::Vector2 Spiral::get_xy(double s) const {
  double xs_spiral, ys_spiral, as_spiral;
  odrSpiral(s - s0_ + s0_spiral_, c_dot_, &xs_spiral, &ys_spiral, &as_spiral);

  const double hdg = hdg0_ - a0_spiral_;
  const double xt = (std::cos(hdg) * (xs_spiral - x0_spiral_)) - (std::sin(hdg) * (ys_spiral - y0_spiral_)) + x0_;
  const double yt = (std::sin(hdg) * (xs_spiral - x0_spiral_)) + (std::cos(hdg) * (ys_spiral - y0_spiral_)) + y0_;
  return {xt, yt};
}

maliput::math::Vector2 Spiral::get_grad(double s) const {
  double xs_spiral, ys_spiral, as_spiral;
  odrSpiral(s - s0_ + s0_spiral_, c_dot_, &xs_spiral, &ys_spiral, &as_spiral);
  const double hdg = as_spiral + hdg0_ - a0_spiral_;
  const double dx = std::cos(hdg);
  const double dy = std::sin(hdg);
  return {{dx, dy}};
}

} // namespace odr

namespace malidrive {
namespace road_curve {


OdrSpiralGroundCurve::OdrSpiralGroundCurve(
    double linear_tolerance, const maliput::math::Vector2& xy0, double heading0, double curvature0,
    double curvature1, double arc_length, double p0, double p1) :
    linear_tolerance_(linear_tolerance), spiral_(p0, xy0.x(), xy0.y(), heading0, arc_length, curvature0, curvature1),
    xy0_(xy0), heading0_(heading0), curvature0_(curvature0), curvature1_(curvature1), arc_length_(arc_length),
    p0_(p0), p1_(p1), validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance_, GroundCurve::kEpsilon)),
    k_dot_((curvature1 - curvature0) / arc_length), norm_(1.), spiral_p0_(1.), spiral_heading0_(0.),
    spiral_xy0_(0., 0.) {
    
}

double OdrSpiralGroundCurve::DoPFromP(double xodr_p) const {
  return validate_p_(xodr_p);
}

maliput::math::Vector2 OdrSpiralGroundCurve::DoG(double p) const {
  p = validate_p_(p);
  return spiral_.get_xy(p);
}
maliput::math::Vector2 OdrSpiralGroundCurve::DoGDot(double p) const {
  p = validate_p_(p);
  return spiral_.get_grad(p);
}

namespace {

// @brief Find the best p parameter for @p curve that maps a point in the 2D INERTIAL-Frame that minimizes
// the distance to @p point.
// @details The function implements a binary search within a range trying to minimize the distance against
// @p point. The range is split into equal @p partitions. The search can be early terminated if any of the
// extents of the range at each iteration is within @p tolerance.
// @param curve The SpiralGoundCurve to use.
// @param point The point in the 2D INERTIAL-Frame to find a p parameter value for.
// @param p0 The p parameter start range value. It must be non-negative.
// @param p1 The p parameter end range value. It must be greater than @p p0 by GroundCurve::kEpsilon.
// @param partitions The number of partitions of the range [@p p0; @p p1]. It must be greater or equal to 2.
// @param tolerance The tolereance to early stop the search. It must be non-negative.
// @return The p parameter value that minimizes the distance against @p point.
double FindBestPParameter(const OdrSpiralGroundCurve& curve, const maliput::math::Vector2& point, double p0, double p1,
                          size_t partitions, double tolerance) {
  MALIPUT_THROW_UNLESS(p0 >= 0.);
  MALIPUT_THROW_UNLESS(p1 - p0 >= GroundCurve::kEpsilon);
  MALIPUT_THROW_UNLESS(partitions >= 2u);
  MALIPUT_THROW_UNLESS(tolerance >= 0.);

  const size_t kIterations{2u * static_cast<size_t>(std::ceil(std::log(partitions))) + 1u};
  // Initialize the list of p values.
  std::vector<double> p_values(partitions);
  const double step = (p1 - p0) / static_cast<double>(partitions - 1u);
  for (size_t i = 0u; i < partitions - 1u; ++i) {
    p_values[i] = p0 + static_cast<double>(i) * step;
  }
  p_values[partitions - 1u] = p1;

  // Find the p-value that minimizes the distance to the target point using a binary search
  // across the list of p values.
  size_t start_i{0u};
  size_t end_i{partitions - 1u};
  double start_distance{std::numeric_limits<double>::max()};
  double end_distance{std::numeric_limits<double>::max()};
  for (size_t iter = 0u; iter < kIterations; ++iter) {
    const maliput::math::Vector2 start_point = curve.G(p_values[start_i]);
    const maliput::math::Vector2 end_point = curve.G(p_values[end_i]);
    start_distance = (start_point - point).norm();
    end_distance = (end_point - point).norm();
    // Early termination conditions.
    if (start_distance <= tolerance || end_distance <= tolerance) {
      break;
    }
    if ((end_i - start_i) <= 1u) {
      break;
    }
    // Index adjustment for the next iteration.
    if (start_distance <= end_distance) {
      end_i = std::ceil((end_i + start_i) / 2u);
    } else {
      start_i = std::floor((end_i + start_i) / 2u);
    }
  }
  return start_distance <= end_distance ? p_values[start_i] : p_values[end_i];
}

}  // namespace

double OdrSpiralGroundCurve::DoGInverse(const maliput::math::Vector2& point) const {
  static constexpr size_t kMaxIterations{10u};
  static constexpr size_t kPartitionSize{1000u};

  // Initializes the error and the extents of the range to search.
  double start_p = p0_;
  double end_p = p1_;
  double best_p = p0_;
  // Iterates in the search to get a better precision of the p value.
  for (size_t i = 0; i < kMaxIterations; ++i) {
    // Obtains the best p parameter value for the given range.
    best_p = FindBestPParameter(*this, point, start_p, end_p, kPartitionSize, linear_tolerance_);
    // Computes the error, i.e. distance to the point to match. When the error
    // is less or equal to linear_tolerance, there is no need to continue iterating.
    if ((G(best_p) - point).norm() <= linear_tolerance_) {
      return best_p;
    }
    // Creates a new extent whose range is step. Note that FindBestPParameter
    // returns the minimum out of the two possible values in the range.
    const double step = (end_p - start_p) / static_cast<double>(kPartitionSize);
    start_p = std::max(p0_, best_p - step);
    end_p = std::min(p1_, best_p + step);
    // When the range has been constrained beyond the GroundCurve::kEpsilon, we should
    // return to avoid further numerical error even if the parameter is not good enough.
    if ((end_p - start_p) < GroundCurve::kEpsilon) {
      return best_p;
    }
  }
  
  return best_p;
}

double OdrSpiralGroundCurve::DoHeading(double p) const {
  const maliput::math::Vector2 tangent = GDot(p);
  return std::atan2(tangent.y(), tangent.x());
}

double OdrSpiralGroundCurve::DoHeadingDot(double p) const {
  p = validate_p_(p);
  const double delta_p = 2.0 * GroundCurve::kEpsilon;
  const double next_p = p + delta_p;
  const double heading_p = Heading(p);
  const double next_heading_p = Heading(next_p);
  return (next_heading_p - heading_p) / delta_p;
}

}  // namespace road_curve
}  // namespace malidrive