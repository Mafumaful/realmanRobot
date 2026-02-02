#ifndef ARM_CONTROLLER__CUBIC_SPLINE_HPP_
#define ARM_CONTROLLER__CUBIC_SPLINE_HPP_

#include <vector>

namespace arm_controller
{

class CubicSpline
{
public:
  CubicSpline() = default;
  ~CubicSpline() = default;

  void setWaypoints(
    const std::vector<double> & times,
    const std::vector<std::vector<double>> & positions);

  bool sample(double t, std::vector<double> & position, std::vector<double> & velocity);

  double getDuration() const;

private:
  void computeCoefficients();

  std::vector<double> times_;
  std::vector<std::vector<double>> positions_;
  std::vector<std::vector<std::vector<double>>> coeffs_;
  size_t num_joints_;
  bool computed_;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__CUBIC_SPLINE_HPP_
