/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ADMISSIBLE_GTSAM_SYMBOLS_H
#define ADMISSIBLE_GTSAM_SYMBOLS_H

namespace graph_msf {

// Compile Time Functions and Constants for Admissible GTSAM Symbols --------------------------------
// Admissible SYMBOL_CHARs
constexpr char ADMISSIBLE_DYNAMIC_SYMBOL_CHARS[4] = {'r', 'c', 'd', 'l'};
constexpr int ADMISSIBLE_DYNAMIC_STATE_DIM[sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS)] = {6, 6, 3, 3};

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

// Count number of n-D states
template <int n>
constexpr int countNDStates() {
  int count = 0;
  for (int i = 0; i < sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS); ++i) {
    if (ADMISSIBLE_DYNAMIC_STATE_DIM[i] == n) {
      count++;
    }
  }
  return count;
}

// Get Symbol Array of nD-States
template <int n, int numStates>
constexpr std::array<char, numStates> getSymbolArrayForNDStates() {
  std::array<char, numStates> symbols{};
  int count = 0;
  for (int i = 0; i < sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS); ++i) {
    if (ADMISSIBLE_DYNAMIC_STATE_DIM[i] == n) {
      symbols[count] = ADMISSIBLE_DYNAMIC_SYMBOL_CHARS[i];
      count++;
    }
  }
  return symbols;
}

// Helper Function to create an array of size N with all elements set to value
template <typename T, size_t N>
constexpr auto makeValuedArray(T value) -> std::array<T, N> {
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

// Runtime Helper Functions for Admissible GTSAM Symbols --------------------------------
// Check whether symbol is in provided character array
template <int ARRAY_SIZE>
inline bool isCharInCharArray(const char c, std::array<char, ARRAY_SIZE> charArray) {
  for (int i = 0; i < ARRAY_SIZE; ++i) {
    if (c == charArray[i]) {
      return true;
    }
  }
  return false;
}

}  // namespace graph_msf

#endif  // ADMISSIBLE_GTSAM_SYMBOLS_H
