/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_TERMINAL_H
#define GMSF_TERMINAL_H

#include <iomanip>
#include <iostream>
#include <string_view>

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"
#define BLUE_START "\033[94m"
#define MAGENTA_START "\033[95m"
#define GOLD_START "\033[38;5;220m"

/*// Constexpr Macros
constexpr std::string_view GREEN_START = "\033[92m";
constexpr std::string_view YELLOW_START = "\033[33m";
constexpr std::string_view RED_START = "\033[31m";
constexpr std::string_view COLOR_END = "\033[0m";*/

#endif  // GMSF_TERMINAL_H
