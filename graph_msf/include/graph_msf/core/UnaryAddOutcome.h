/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARY_ADD_OUTCOME_H
#define GRAPH_MSF_UNARY_ADD_OUTCOME_H

namespace graph_msf {

enum class UnaryAddStatus { Added, DeferredFuture, Dropped };

struct UnaryAddOutcome {
  UnaryAddStatus status = UnaryAddStatus::Dropped;
  double futureLeadSeconds = 0.0;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARY_ADD_OUTCOME_H
