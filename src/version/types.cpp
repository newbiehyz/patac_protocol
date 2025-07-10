//
#include "version/types.h"

#include <iostream>

namespace autodrive {

std::string GetVersion(int *major, int *minor, int *patch) {
  if (major) {
    *major = HYPERSAR_VERSION_MAJOR;
  }
  if (minor) {
    *minor = HYPERSAR_VERSION_MAJOR;
  }
  if (patch) {
    *patch = HYPERSAR_VERSION_MAJOR;
  }
  std::string version_str = std::string("v") +
      std::to_string(HYPERSAR_VERSION_MAJOR) + "." +
      std::to_string(HYPERSAR_VERSION_MAJOR) + "." +
      std::to_string(HYPERSAR_VERSION_MAJOR);
  return version_str;
}

std::string GetSoVersion(int *major, int *minor, int *patch) {
# define _HSAR_VER_STR(m, n, p, x, y, z) "product version: " #m "." #n "." #p \
                                " ; soname version: " #x "." #y "." #z
# define HSAR_VER_STR(m, n, p, x, y, z)  _HSAR_VER_STR(m, n, p, x, y, z)

  if (major) {
    *major = HYPERSAR_SONAME_VERSION_MAJOR;
  }
  if (minor) {
    *minor = HYPERSAR_SONAME_VERSION_MINOR;
  }
  if (patch) {
    *patch = HYPERSAR_SONAME_VERSION_PATCH;
  }

  return std::string(HSAR_VER_STR(HYPERSAR_VERSION_MAJOR,
      HYPERSAR_VERSION_MINOR,
      HYPERSAR_VERSION_PATCH,
      HYPERSAR_SONAME_VERSION_MAJOR,
      HYPERSAR_SONAME_VERSION_MINOR,
      HYPERSAR_SONAME_VERSION_PATCH));
}

void PrintVersionList() {
  std::cout << "Product Release Version <=> SONAME version" << std::endl;
  std::cout << "v2.0.0                  <=> 1.0.0" << std::endl;
}

}