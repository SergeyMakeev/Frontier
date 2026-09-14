#pragma once

#include <string>

namespace city {

// Read once at startup. Missing platform data is reported explicitly.
std::string cpuModel();

} // namespace city
