/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */
#pragma once

#include <string>
#include <tf2/LinearMath/Transform.h>

namespace graph_msf
{

  class ElementToRoot final
  {
  public:
    /// Constructor
    explicit ElementToRoot(const tf2::Transform &T, const std::string &rootName_, const std::string &elementName_)
        : T_root_element(T), rootName(rootName_), elementName(elementName_) {}

    tf2::Transform T_root_element; ///< The tf2 Transform from root to element
    std::string rootName;          ///< The name of the root element to which this link is attached
    std::string elementName;       ///< The name of the element
  };

} // namespace graph_msf
