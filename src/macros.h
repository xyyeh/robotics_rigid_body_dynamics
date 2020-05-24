#include <iostream>

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define ASSERT(condition, message)                                         \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << "Assertion `" #condition "` failed in " << __FILENAME__ \
                << " line " << __LINE__ << ": [INFO] " << message          \
                << std::endl;                                              \
      std::terminate();                                                    \
    }                                                                      \
  } while (false)

