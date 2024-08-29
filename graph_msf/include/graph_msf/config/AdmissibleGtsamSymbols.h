/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ADMISSIBLE_GTSAM_SYMBOLS_H
#define ADMISSIBLE_GTSAM_SYMBOLS_H

namespace graph_msf {

// Admissible SYMBOL_CHARs
constexpr char ADMISSIBLE_DYNAMIC_SYMBOL_CHARS[4] = {'r', 'c', 'd', 'l'};

// Get index of letter at compile time
template <char SYMBOL_CHAR>
constexpr int getSymbolIndex() {
  for (int i = 0; i < sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS); ++i) {
    if (SYMBOL_CHAR == ADMISSIBLE_DYNAMIC_SYMBOL_CHARS[i]) {
      return i;
    }
  }
  // If not found, return -1
  return -1;
}

// Helper Function to create an array of size N with all elements set to value
template <typename T, size_t N>
constexpr auto make_valued_array(T value) -> std::array<T, N> {
  std::array<T, N> a{};
  for (auto& x : a) x = value;
  return a;
}

// Function to check whether it is an admissible symbol shorthand
template <char SYMBOL_CHAR>
constexpr bool isAdmissibleSymbol() {
  for (int i = 0; i < sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS); ++i) {
    if (SYMBOL_CHAR == ADMISSIBLE_DYNAMIC_SYMBOL_CHARS[i]) {
      return true;
    }
  }
  return false;
}

// Use static_assert to include SYMBOL_CHAR in the message
template <char SYMBOL_CHAR>
constexpr void checkSymbol() {
  static_assert(isAdmissibleSymbol<SYMBOL_CHAR>(), "Symbol is not admissible");
};

}  // namespace graph_msf

#endif  // ADMISSIBLE_GTSAM_SYMBOLS_H
