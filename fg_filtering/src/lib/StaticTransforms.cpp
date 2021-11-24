//
// Created by nubertj on 30.06.21.
//

#include "fg_filtering/StaticTransforms.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace compslam_se {

StaticTransforms::StaticTransforms(ros::NodeHandle& privateNode) {
  ROS_INFO("Static Transforms container initializing...");
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
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdfModel_, tree)) {
    throw std::runtime_error("Failed to extract kdl tree from robot description");
  }

  KDL::SegmentMap segments_map = tree.getSegments();

  // walk the tree and add segments to segments_
  segments_.clear();
  segments_fixed_.clear();
  addChildren(tree.getRootSegment());
}

void StaticTransforms::addChildren(const KDL::SegmentMap::const_iterator segment) {
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (urdfModel_.getJoint(child.getJoint().getName()) &&
          urdfModel_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
    }
    addChildren(children[i]);
  }
}

void StaticTransforms::findTransformations() {
  ROS_WARN("Looking up transformations in URDF model...");

  for (const std::pair<const std::string, SegmentPair>& seg : segments_fixed_) {
    if (seg.second.tip == getImuRooftopFrame()) {
      tf_T_Ic_C_ = kdlToTransform(seg.second.segment.pose(0));
      tf_T_C_Ic_ = tf_T_Ic_C_.inverse();
      ROS_WARN_STREAM("IMU frame with respect to cabin frame: t=[" << tf_T_C_Ic_.getOrigin().x() << ", " << tf_T_C_Ic_.getOrigin().y()
                                                                   << ", " << tf_T_C_Ic_.getOrigin().z() << "]");
    } else if (seg.second.tip == getLidarFrame()) {
      tf_T_L_C_ = kdlToTransform(seg.second.segment.pose(0));
      tf_T_C_L_ = tf_T_L_C_.inverse();
      tf_T_L_Ic_ = tf_T_L_C_ * tf_T_C_Ic_;
      ROS_WARN_STREAM("IMU frame with respect to LiDAR frame: t=[" << tf_T_L_Ic_.getOrigin().x() << ", " << tf_T_L_Ic_.getOrigin().y()
                                                                   << ", " << tf_T_L_Ic_.getOrigin().z() << "]");
    } else if (seg.second.tip == getLeftGnssFrame()) {
      tf_T_GnssL_C_ = kdlToTransform(seg.second.segment.pose(0));
      tf_T_Cabin_GnssL_ = tf_T_GnssL_C_.inverse();
      ROS_WARN_STREAM("Left GNSS with respect to Cabin frame: t=[" << tf_T_Cabin_GnssL_.getOrigin().x() << ", "
                                                                   << tf_T_Cabin_GnssL_.getOrigin().y() << ", "
                                                                   << tf_T_Cabin_GnssL_.getOrigin().z() << "]");
    } else if (seg.second.tip == getRightGnssFrame()) {
      tf_T_GnssR_C_ = kdlToTransform(seg.second.segment.pose(0));
      tf_T_C_GnssR_ = tf_T_GnssR_C_.inverse();
      ROS_WARN_STREAM("Right GNSS to Cabin translation: [" << tf_T_C_GnssR_.getOrigin().x() << ", " << tf_T_C_GnssR_.getOrigin().y() << ", "
                                                           << tf_T_C_GnssR_.getOrigin().z() << "]");
    }
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
