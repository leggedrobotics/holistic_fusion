#ifndef ELEMENTTOROOT_H
#define ELEMENTTOROOT_H

namespace compslam_se {

class ElementToRoot final {
 public:
  /// Constructor
  explicit ElementToRoot(const tf::Transform& T, const std::string& rootName_, const std::string& elementName_)
      : T_root_element(T), rootName(rootName_), elementName(elementName_) {}

  tf::Transform T_root_element;  ///< The KDL segment
  std::string rootName;          ///< The name of the root element to which this link is attached
  std::string elementName;       ///< The name of the element
};

}  // namespace compslam_se

#endif  // ELEMENTTOROOT_H
