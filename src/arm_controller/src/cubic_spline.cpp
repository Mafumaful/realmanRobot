#include "arm_controller/cubic_spline.hpp"
#include <cmath>
#include <algorithm>

namespace arm_controller
{

void CubicSpline::setWaypoints(
  const std::vector<double> & times,
  const std::vector<std::vector<double>> & positions)
{
  times_ = times;
  positions_ = positions;
  computed_ = false;

  if (times_.size() < 2 || positions_.empty()) {
    return;
  }

  num_joints_ = positions_[0].size();
  computeCoefficients();
}

double CubicSpline::getDuration() const
{
  if (times_.empty()) {
    return 0.0;
  }
  return times_.back() - times_.front();
}

void CubicSpline::computeCoefficients()
{
  size_t n = times_.size() - 1;
  coeffs_.resize(num_joints_);

  for (size_t j = 0; j < num_joints_; ++j) {
    coeffs_[j].resize(n);

    std::vector<double> h(n), alpha(n);
    for (size_t i = 0; i < n; ++i) {
      h[i] = times_[i + 1] - times_[i];
    }

    for (size_t i = 1; i < n; ++i) {
      alpha[i] = (3.0 / h[i]) * (positions_[i + 1][j] - positions_[i][j]) -
                 (3.0 / h[i - 1]) * (positions_[i][j] - positions_[i - 1][j]);
    }

    std::vector<double> l(n + 1), mu(n + 1), z(n + 1);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (size_t i = 1; i < n; ++i) {
      l[i] = 2.0 * (times_[i + 1] - times_[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n] = 1.0;
    z[n] = 0.0;

    std::vector<double> c(n + 1), b(n), d(n);
    c[n] = 0.0;

    for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
      c[i] = z[i] - mu[i] * c[i + 1];
      b[i] = (positions_[i + 1][j] - positions_[i][j]) / h[i] -
             h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
    }

    for (size_t i = 0; i < n; ++i) {
      coeffs_[j][i] = {positions_[i][j], b[i], c[i], d[i]};
    }
  }

  computed_ = true;
}

bool CubicSpline::sample(double t, std::vector<double> & position, std::vector<double> & velocity)
{
  if (!computed_ || times_.size() < 2) {
    return false;
  }

  t = std::max(times_.front(), std::min(t, times_.back()));

  size_t seg = 0;
  for (size_t i = 0; i < times_.size() - 1; ++i) {
    if (t >= times_[i] && t <= times_[i + 1]) {
      seg = i;
      break;
    }
  }

  double dt = t - times_[seg];
  position.resize(num_joints_);
  velocity.resize(num_joints_);

  for (size_t j = 0; j < num_joints_; ++j) {
    const auto & c = coeffs_[j][seg];
    position[j] = c[0] + c[1] * dt + c[2] * dt * dt + c[3] * dt * dt * dt;
    velocity[j] = c[1] + 2.0 * c[2] * dt + 3.0 * c[3] * dt * dt;
  }

  return true;
}

}  // namespace arm_controller
