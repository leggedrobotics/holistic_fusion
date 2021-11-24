//
// Created by nubertj on 30.06.21.
//

#include "fg_filtering/StaticTransforms.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace compslam_se {

StaticTransforms::StaticTransforms(ros::NodeHandle& privateNode) {
  ROS_WARN("Static Transforms container initializing...");
  std::string urdfDescriptionName;
  if (privateNode.getParam("launch/description_name", urdfDescriptionName)) {
    ROS_INFO_STREAM("FactorGraphFiltering - URDF-Description-Name: " << urdfDescriptionName);
    privateNode.getParam(std::string("/") + urdfDescriptionName, urdfDescription_);
    // urdfDescription_ = sParam;
    if (urdfDescription_.empty()) {
      ROS_ERROR("Could not load description!");
      return;
    }
  } else {
    ROS_ERROR("FactorGraphFiltering - urdf description not set.");
    throw std::runtime_error("Robot description must be provided in rosparams.");
  }
  // load excavator model from URDF
  double timeStep;
  urdfModel_.initParam(urdfDescriptionName);

  // Initialize the KDL tree
  if (!kdl_parser::treeFromUrdfModel(urdfModel_, tree_)) {
    throw std::runtime_error("Failed to extract kdl tree from robot description");
  }

  KDL::SegmentMap segments_map = tree_.getSegments();
  KDL::Chain chain;

  // walk the tree and add segments to segments_
  segments_.clear();
  getRootTransformations(tree_.getRootSegment());
}

void StaticTransforms::getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName) {
  const std::string& elementName = GetTreeElementSegment(element->second).getName();
  if (rootName == "") {
    rootName = elementName;
  }
  ROS_WARN_STREAM("Root of KDL tree: " << rootName);

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(element->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    // Go through children
    const KDL::SegmentMap::const_iterator child = children[i];
    // Get kinematic chain from current child to root
    KDL::Chain chain;
    ROS_WARN_STREAM("Child name: " << child->second.segment.getName());
    tree_.getChain(rootName, child->second.segment.getName(), chain);
    // Compute trafo to root
    KDL::Frame frameToRoot = chain.getSegment(0).getFrameToTip();
    double x, y, z, w;
    frameToRoot.M.GetQuaternion(x, y, z, w);
    tf::Quaternion q(x, y, z, w);
    tf::Vector3 t(frameToRoot.p[0], frameToRoot.p[1], frameToRoot.p[2]);
    // Write into buffer
    ElementToRoot T_root_element(tf::Transform(q, t), rootName, child->second.segment.getName());
    // Insert into segments
    segments_.insert(std::make_pair(child->second.segment.getName(), T_root_element));
    // Call recursively
    getRootTransformations(child, rootName);
  }
}

void StaticTransforms::findTransformations() {
  ROS_WARN("Looking up transformations in URDF model...");

  // Get static transforms within cabin
  for (const std::pair<const std::string, ElementToRoot>& seg : segments_) {
    if (seg.second.elementName == getImuRooftopFrame()) {
      tf_T_C_Ic_ = seg.second.T_root_element;
      tf_T_Ic_C_ = tf_T_C_Ic_.inverse();
      ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_C_Ic_.getOrigin().x() << ", "
                                             << tf_T_C_Ic_.getOrigin().y() << ", " << tf_T_C_Ic_.getOrigin().z() << "]");
    } else if (seg.second.elementName == getLidarFrame()) {
      tf_T_C_L_ = seg.second.T_root_element;
      tf_T_L_C_ = tf_T_C_L_.inverse();
      tf_T_L_Ic_ = tf_T_L_C_ * tf_T_C_Ic_;
      ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_L_Ic_.getOrigin().x() << ", "
                                             << tf_T_L_Ic_.getOrigin().y() << ", " << tf_T_L_Ic_.getOrigin().z() << "]");
    } else if (seg.second.elementName == getLeftGnssFrame()) {
      tf_T_C_GnssL_ = seg.second.T_root_element;
      tf_T_GnssL_C_ = tf_T_C_GnssL_.inverse();
      ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_C_GnssL_.getOrigin().x()
                                             << ", " << tf_T_C_GnssL_.getOrigin().y() << ", " << tf_T_C_GnssL_.getOrigin().z() << "]");
    } else if (seg.second.elementName == getRightGnssFrame()) {
      tf_T_C_GnssR_ = seg.second.T_root_element;
      tf_T_GnssR_C_ = tf_T_C_GnssR_.inverse();
      ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": [" << tf_T_C_GnssR_.getOrigin().x() << ", "
                                             << tf_T_C_GnssR_.getOrigin().y() << ", " << tf_T_C_GnssR_.getOrigin().z() << "]");
    }
  }
  // Get offset between cabin and base
  std::shared_ptr<const urdf::Joint> cabinJoint = urdfModel_.getJoint("J_TURN");
  if (cabinJoint) {
    BC_Z_offset_ = cabinJoint->parent_to_joint_origin_transform.position.z;
    ROS_WARN_STREAM("Cabin to Base translation: " << BC_Z_offset_);
  } else {
    throw std::runtime_error("[M545 tf publisher] Did not find cabin turn joint in model.");
  }
}

tf::Transform StaticTransforms::kdlToTransform(const KDL::Frame& k) {
  tf::Transform tf_T;
  tf_T.setOrigin(tf::Vector3(k.p.x(), k.p.y(), k.p.z()));
  double qx, qy, qz, qw;
  k.M.GetQuaternion(qx, qy, qz, qw);
  tf_T.setRotation(tf::Quaternion(qx, qy, qz, qw));

  return tf_T;
}

}  // namespace compslam_se
