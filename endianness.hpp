#pragma once

// https://en.cppreference.com/mwiki/index.php?title=cpp/types/endian&oldid=154532#Possible_implementation
enum class endian {
#if defined(_MSC_VER) && !defined(__clang__)
  little = 0,
  big = 1,
  native = little
#else
  little = __ORDER_LITTLE_ENDIAN__,
  big = __ORDER_BIG_ENDIAN__,
  native = __BYTE_ORDER__
#endif
};
