#include "hardware_info.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <string_view>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/sysctl.h>
#endif

namespace city {
namespace {

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n\0", 0, 5);
    if (first == std::string::npos)
        return {};
    const auto last = value.find_last_not_of(" \t\r\n\0", std::string::npos, 5);
    return value.substr(first, last - first + 1);
}

#if defined(__linux__)
std::string readFile(const char* path)
{
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), {}};
}

std::string armCoreName(unsigned implementer, unsigned part)
{
    if (implementer == 0x41)
    {
        switch (part)
        {
        case 0xd03: return "Cortex-A53";
        case 0xd04: return "Cortex-A35";
        case 0xd05: return "Cortex-A55";
        case 0xd07: return "Cortex-A57";
        case 0xd08: return "Cortex-A72";
        case 0xd09: return "Cortex-A73";
        case 0xd0a: return "Cortex-A75";
        case 0xd0b: return "Cortex-A76";
        case 0xd0d: return "Cortex-A77";
        case 0xd41: return "Cortex-A78";
        case 0xd46: return "Cortex-A510";
        case 0xd47: return "Cortex-A710";
        }
    }
    char name[80];
    std::snprintf(name, sizeof(name), "ARM implementer 0x%x / part 0x%x",
                  implementer, part);
    return name;
}

std::string linuxCpuModel(std::string_view cpuInfo)
{
    std::istringstream input{std::string(cpuInfo)};
    std::map<std::string, unsigned> cores;
    std::string hardware;
    unsigned implementer = 0;
    std::string line;
    while (std::getline(input, line))
    {
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        const auto key = trim(line.substr(0, colon));
        const auto value = trim(line.substr(colon + 1));
        if (key == "model name" && !value.empty())
            return value;
        if (key == "processor")
            implementer = 0;
        else if (key == "Hardware")
            hardware = value;
        else if (key == "CPU implementer")
        {
            implementer = 0;
            std::sscanf(value.c_str(), "%x", &implementer);
        }
        else if (key == "CPU part")
        {
            unsigned part = 0;
            if (std::sscanf(value.c_str(), "%x", &part) == 1)
                ++cores[armCoreName(implementer, part)];
        }
    }
    std::string model;
    for (const auto& [name, count] : cores)
    {
        if (!model.empty()) model += " + ";
        model += name + " x" + std::to_string(count);
    }
    return model.empty() ? hardware : model;
}
#endif

} // namespace

std::string cpuModel()
{
#if defined(_WIN32)
    char model[256]{};
    DWORD size = sizeof(model);
    if (RegGetValueA(HKEY_LOCAL_MACHINE,
                     "HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0",
                     "ProcessorNameString", RRF_RT_REG_SZ, nullptr,
                     model, &size) == ERROR_SUCCESS)
        if (auto name = trim(model); !name.empty()) return name;
#elif defined(__APPLE__)
    char model[256]{};
    size_t size = sizeof(model);
    if (sysctlbyname("machdep.cpu.brand_string", model, &size, nullptr, 0) == 0)
        if (auto name = trim(model); !name.empty()) return name;
#elif defined(__linux__)
    if (auto name = linuxCpuModel(readFile("/proc/cpuinfo")); !name.empty())
        return name;
#endif
    return "Model unavailable";
}

} // namespace city
