#include "arm_controller/kdl_parser.hpp"

#include <kdl/frames_io.hpp>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

namespace arm_controller
{

KDL::Vector toKdl(const urdf::Vector3 & v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

KDL::Rotation toKdl(const urdf::Rotation & r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

KDL::Frame toKdl(const urdf::Pose & p)
{
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

KDL::Joint toKdlJoint(const urdf::JointSharedPtr & jnt)
{
  KDL::Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

  switch (jnt->type) {
    case urdf::Joint::FIXED:
      return KDL::Joint(jnt->name, KDL::Joint::Fixed);
    case urdf::Joint::REVOLUTE:
      return KDL::Joint(jnt->name,
        F_parent_jnt.p,
        F_parent_jnt.M * toKdl(jnt->axis),
        KDL::Joint::RotAxis);
    case urdf::Joint::CONTINUOUS:
      return KDL::Joint(jnt->name,
        F_parent_jnt.p,
        F_parent_jnt.M * toKdl(jnt->axis),
        KDL::Joint::RotAxis);
    case urdf::Joint::PRISMATIC:
      return KDL::Joint(jnt->name,
        F_parent_jnt.p,
        F_parent_jnt.M * toKdl(jnt->axis),
        KDL::Joint::TransAxis);
    default:
      return KDL::Joint(jnt->name, KDL::Joint::Fixed);
  }
}

KDL::RigidBodyInertia toKdlInertia(const urdf::InertialSharedPtr & i)
{
  if (!i) {
    return KDL::RigidBodyInertia::Zero();
  }

  KDL::Frame origin = toKdl(i->origin);
  return origin.M * KDL::RigidBodyInertia(
    i->mass,
    origin.p,
    KDL::RotationalInertia(
      i->ixx, i->iyy, i->izz,
      i->ixy, i->ixz, i->iyz));
}

bool addChildrenToTree(
  const urdf::LinkConstSharedPtr & root,
  KDL::Tree & tree)
{
  for (size_t i = 0; i < root->child_joints.size(); ++i) {
    auto joint = root->child_joints[i];
    auto child = root->child_links[i];

    KDL::Frame F_parent_jnt = toKdl(joint->parent_to_joint_origin_transform);
    KDL::Joint kdl_jnt = toKdlJoint(joint);
    KDL::RigidBodyInertia I = toKdlInertia(child->inertial);
    KDL::Segment sgm(child->name, kdl_jnt, F_parent_jnt, I);

    tree.addSegment(sgm, root->name);

    if (!addChildrenToTree(child, tree)) {
      return false;
    }
  }
  return true;
}

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr & robot_model, KDL::Tree & tree)
{
  auto root = robot_model->getRoot();
  if (!root) {
    return false;
  }

  tree = KDL::Tree(root->name);
  return addChildrenToTree(root, tree);
}

}  // namespace arm_controller
