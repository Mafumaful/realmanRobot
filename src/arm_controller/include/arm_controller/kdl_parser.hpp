#ifndef ARM_CONTROLLER__KDL_PARSER_HPP_
#define ARM_CONTROLLER__KDL_PARSER_HPP_

#include <string>
#include <kdl/tree.hpp>
#include <urdf/model.h>

namespace arm_controller
{

bool treeFromUrdfModel(const urdf::Model & robot_model, KDL::Tree & tree);

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__KDL_PARSER_HPP_
