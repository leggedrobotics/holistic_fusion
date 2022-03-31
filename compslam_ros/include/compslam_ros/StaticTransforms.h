#ifndef M545StaticTransforms_H
#define M545StaticTransforms_H

// ROS
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Workspace
#include "compslam_se/StaticTransforms.h"

namespace compslam_ros {

class StaticTransforms : public compslam_se::StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode) {
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
      ROS_ERROR("M545StaticTransforms - urdf description not set.");
      std::runtime_error("Robot description must be provided in rosparams.");
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

 private:
  KDL::Tree tree_;

  void getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName = "") {
    const std::string& elementName = GetTreeElementSegment(element->second).getName();
    if (rootName == "") {
      rootName = elementName;
    }
    // Iterate through children
    std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(element->second);
    for (unsigned int i = 0; i < children.size(); i++) {
      // Go through children
      const KDL::SegmentMap::const_iterator child = children[i];
      // Get kinematic chain from current child to root
      KDL::Chain chain;
      tree_.getChain(rootName, child->second.segment.getName(), chain);
      // Compute trafo to root
      tf::Transform T_root_element = tf::Transform::getIdentity();
      for (int i = 0; i < chain.getNrOfSegments(); ++i) {
        KDL::Frame frameToRoot = chain.getSegment(i).getFrameToTip();
        double x, y, z, w;
        frameToRoot.M.GetQuaternion(x, y, z, w);
        tf::Quaternion q(x, y, z, w);
        tf::Vector3 t(frameToRoot.p[0], frameToRoot.p[1], frameToRoot.p[2]);
        T_root_element = T_root_element * tf::Transform(q, t);
      }
      // Write into buffer
      compslam_se::ElementToRoot elementToRoot(T_root_element, rootName, child->second.segment.getName());
      // Insert into segments
      segments_.insert(std::make_pair(child->second.segment.getName(), elementToRoot));
      // Call recursively
      getRootTransformations(child, rootName);
    }
  }

  void findTransformations() {
    ROS_WARN("Looking up transformations in URDF model...");
    // Variable needed for conversion
    tf::Transform tf_T_C_B;
    // Get offset between cabin and base
    std::shared_ptr<const urdf::Joint> cabinJoint = urdfModel_.getJoint("J_TURN");
    if (cabinJoint) {
      BC_Z_offset_ = cabinJoint->parent_to_joint_origin_transform.position.z;
      tf_T_C_B = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, BC_Z_offset_)).inverse();
      ROS_WARN_STREAM("Cabin to Base translation: " << BC_Z_offset_);
    } else {
      throw std::runtime_error("[M545 tf publisher] Did not find cabin turn joint in model.");
    }
    // Get static transforms within cabin
    bool IccPresent = false;
    for (const std::pair<const std::string, compslam_se::ElementToRoot>& seg : segments_) {
      if (seg.second.elementName == getImuRooftopFrame()) {
        tf::Transform tf_T_B_Icr = seg.second.T_root_element;
        tf_T_C_Icr_ = tf_T_C_B * tf_T_B_Icr;
        tf_T_Icr_C_ = tf_T_C_Icr_.inverse();
        ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_C_Icr_.getOrigin().x()
                                               << ", " << tf_T_C_Icr_.getOrigin().y() << ", " << tf_T_C_Icr_.getOrigin().z() << "]");
      } else if (seg.second.elementName == getImuCabinFrame()) {
        tf::Transform tf_T_B_Icc = seg.second.T_root_element;
        tf_T_C_Icc_ = tf_T_C_B * tf_T_B_Icc;
        tf_T_Icc_C_ = tf_T_C_Icc_.inverse();
        ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_C_Icc_.getOrigin().x()
                                               << ", " << tf_T_C_Icc_.getOrigin().y() << ", " << tf_T_C_Icc_.getOrigin().z() << "]");
        IccPresent = true;
      }
    }
    // Check which IMU is present
    if (IccPresent) {
      tf_T_Ic_C_ = tf_T_Icc_C_;
      tf_T_C_Ic_ = tf_T_C_Icc_;
      ROS_WARN("Cabin IMU is present.");
    } else {
      tf_T_Ic_C_ = tf_T_Icr_C_;
      tf_T_C_Ic_ = tf_T_C_Icr_;
      ROS_WARN("Only rooftop IMU is present.");
    }

    // Go through remaining transformations
    for (const std::pair<const std::string, compslam_se::ElementToRoot>& seg : segments_) {
      if (seg.second.elementName == getLidarFrame()) {
        tf::Transform tf_T_B_L = seg.second.T_root_element;
        tf_T_C_L_ = tf_T_C_B * tf_T_B_L;
        tf_T_L_C_ = tf_T_C_L_.inverse();
        tf_T_L_Ic_ = tf_T_L_C_ * tf_T_C_Ic_;
        ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_L_Ic_.getOrigin().x()
                                               << ", " << tf_T_L_Ic_.getOrigin().y() << ", " << tf_T_L_Ic_.getOrigin().z() << "]");
      } else if (seg.second.elementName == getLeftGnssFrame()) {
        tf::Transform tf_T_B_GnssL = seg.second.T_root_element;
        tf_T_C_GnssL_ = tf_T_C_B * tf_T_B_GnssL;
        tf_T_GnssL_C_ = tf_T_C_GnssL_.inverse();
        ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": t=[" << tf_T_C_GnssL_.getOrigin().x()
                                               << ", " << tf_T_C_GnssL_.getOrigin().y() << ", " << tf_T_C_GnssL_.getOrigin().z() << "]");
      } else if (seg.second.elementName == getRightGnssFrame()) {
        tf::Transform tf_T_B_GnssR = seg.second.T_root_element;
        tf_T_C_GnssR_ = tf_T_C_B * tf_T_B_GnssR;
        tf_T_GnssR_C_ = tf_T_C_GnssR_.inverse();
        ROS_WARN_STREAM(seg.second.elementName << " with respect to " << seg.second.rootName << ": [" << tf_T_C_GnssR_.getOrigin().x()
                                               << ", " << tf_T_C_GnssR_.getOrigin().y() << ", " << tf_T_C_GnssR_.getOrigin().z() << "]");
      }
    }
  }

  tf::Transform kdlToTransform(const KDL::Frame& k) {
    tf::Transform tf_T;
    tf_T.setOrigin(tf::Vector3(k.p.x(), k.p.y(), k.p.z()));
    double qx, qy, qz, qw;
    k.M.GetQuaternion(qx, qy, qz, qw);
    tf_T.setRotation(tf::Quaternion(qx, qy, qz, qw));

    return tf_T;
  }
};
}  // namespace compslam_ros
#endif  // end M545ESTIMATORGRAPH_H
