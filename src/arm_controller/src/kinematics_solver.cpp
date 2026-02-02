#include "arm_controller/kinematics_solver.hpp"
#include "arm_controller/kdl_parser.hpp"

#include <urdf/model.h>

namespace arm_controller
{

KinematicsSolver::KinematicsSolver()
: initialized_(false)
{
}

bool KinematicsSolver::initialize(const std::string & urdf_string)
{
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_string)) {
    return false;
  }

  if (!treeFromUrdfModel(urdf_model, tree_)) {
    return false;
  }

  // Build chains for each arm
  if (!buildChain("arm_A", "platform_link", "Link_A6")) {
    return false;
  }
  if (!buildChain("arm_B", "platform_link", "Link_B6")) {
    return false;
  }
  if (!buildChain("arm_S", "platform_link", "Link_S6")) {
    return false;
  }

  initialized_ = true;
  return true;
}

bool KinematicsSolver::buildChain(
  const std::string & arm_name,
  const std::string & base_link,
  const std::string & tip_link)
{
  KDL::Chain chain;
  if (!tree_.getChain(base_link, tip_link, chain)) {
    return false;
  }

  chains_[arm_name] = chain;

  // Create FK solver
  fk_solvers_[arm_name] = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);

  // Create IK solver (LMA - Levenberg-Marquardt)
  Eigen::Matrix<double, 6, 1> L;
  L << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
  ik_solvers_[arm_name] = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain, L);

  // Extract joint names and limits
  std::vector<std::string> names;
  std::vector<double> lower, upper;

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed) {
      names.push_back(joint.getName());
      lower.push_back(-3.14);
      upper.push_back(3.14);
    }
  }

  joint_names_[arm_name] = names;
  joint_limits_lower_[arm_name] = lower;
  joint_limits_upper_[arm_name] = upper;

  return true;
}

bool KinematicsSolver::solveFK(
  const std::string & arm_name,
  const std::vector<double> & joint_positions,
  KDL::Frame & end_effector_pose)
{
  if (!initialized_ || fk_solvers_.find(arm_name) == fk_solvers_.end()) {
    return false;
  }

  const auto & chain = chains_[arm_name];
  if (joint_positions.size() != chain.getNrOfJoints()) {
    return false;
  }

  KDL::JntArray q(chain.getNrOfJoints());
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    q(i) = joint_positions[i];
  }

  int result = fk_solvers_[arm_name]->JntToCart(q, end_effector_pose);
  return result >= 0;
}

bool KinematicsSolver::solveIK(
  const std::string & arm_name,
  const KDL::Frame & target_pose,
  const std::vector<double> & seed_positions,
  std::vector<double> & solution)
{
  if (!initialized_ || ik_solvers_.find(arm_name) == ik_solvers_.end()) {
    return false;
  }

  const auto & chain = chains_[arm_name];
  unsigned int nj = chain.getNrOfJoints();

  if (seed_positions.size() != nj) {
    return false;
  }

  KDL::JntArray q_init(nj);
  for (size_t i = 0; i < nj; ++i) {
    q_init(i) = seed_positions[i];
  }

  KDL::JntArray q_out(nj);
  int result = ik_solvers_[arm_name]->CartToJnt(q_init, target_pose, q_out);

  if (result < 0) {
    return false;
  }

  solution.resize(nj);
  for (size_t i = 0; i < nj; ++i) {
    solution[i] = q_out(i);
  }

  return true;
}

std::vector<std::string> KinematicsSolver::getJointNames(const std::string & arm_name) const
{
  auto it = joint_names_.find(arm_name);
  if (it != joint_names_.end()) {
    return it->second;
  }
  return {};
}

std::vector<double> KinematicsSolver::getJointLimitsLower(const std::string & arm_name) const
{
  auto it = joint_limits_lower_.find(arm_name);
  if (it != joint_limits_lower_.end()) {
    return it->second;
  }
  return {};
}

std::vector<double> KinematicsSolver::getJointLimitsUpper(const std::string & arm_name) const
{
  auto it = joint_limits_upper_.find(arm_name);
  if (it != joint_limits_upper_.end()) {
    return it->second;
  }
  return {};
}

}  // namespace arm_controller
