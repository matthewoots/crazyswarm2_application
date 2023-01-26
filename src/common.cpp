#include "common.h"

std::set<std::string> common::extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
    std::set<std::string> result;
    for (const auto &i : parameter_overrides)
    {
        if (i.first.find(pattern) == 0)
        {
            size_t start = pattern.size() + 1;
            size_t end = i.first.find(".", start);
            result.insert(i.first.substr(start, end - start));
        }
    }
    return result;
}