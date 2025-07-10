//
#ifndef AUTODRIVE_VERSION_TYPES_H_
#define AUTODRIVE_VERSION_TYPES_H_

#include "version.h"
#include <string>

namespace autodrive {

std::string GetVersion(int *major = nullptr, int *minor = nullptr,
    int *patch = nullptr);

std::string GetSoVersion(int *major = nullptr, int *minor = nullptr,
    int *patch = nullptr);

void PrintVersionList();

}

#endif