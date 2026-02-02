#ifndef ARM_CONTROLLER__KINEMATICS_SOLVER_HPP_
#define ARM_CONTROLLER__KINEMATICS_SOLVER_HPP_

#include <string>
#include <vector>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace arm_controller
{

class KinematicsSolver
{
public:
  KinematicsSolver();
  ~KinematicsSolver() = default;

  bool initialize(const std::string & urdf_string);

  bool solveFK(
    const std::string & arm_name,
    const std::vector<double> & joint_positions,
    KDL::Frame & end_effector_pose);

  bool solveIK(
    const std::string & arm_name,
    const KDL::Frame & target_pose,
    const std::vector<double> & seed_positions,
    std::vector<double> & solution);

  std::vector<std::string> getJointNames(const std::string & arm_name) const;
  std::vector<double> getJointLimitsLower(const std::string & arm_name) const;
  std::vector<double> getJointLimitsUpper(const std::string & arm_name) const;

private:
  bool buildChain(
    const std::string & arm_name,
    const std::string & base_link,
    const std::string & tip_link);

  KDL::Tree tree_;
  std::map<std::string, KDL::Chain> chains_;
  std::map<std::string, std::unique_ptr<KDL::ChainFkSolverPos_recursive>> fk_solvers_;
  std::map<std::string, std::unique_ptr<KDL::ChainIkSolverPos_LMA>> ik_solvers_;
  std::map<std::string, std::vector<std::string>> joint_names_;
  std::map<std::string, std::vector<double>> joint_limits_lower_;
  std::map<std::string, std::vector<double>> joint_limits_upper_;

  bool initialized_;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__KINEMATICS_SOLVER_HPP_
