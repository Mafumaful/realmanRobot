#ifndef ARM_CONTROLLER__KDL_PARSER_HPP_
#define ARM_CONTROLLER__KDL_PARSER_HPP_

#include <string>
#include <memory>
#include <kdl/tree.hpp>
#include <urdf_model/model.h>

namespace arm_controller
{

bool treeFromUrdfModel(const std::shared_ptr<urdf::ModelInterface> & robot_model, KDL::Tree & tree);

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__KDL_PARSER_HPP_
