//  Copyright (c) 2026 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#include "ros2_control_crospi/robot_description_loader.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>

#include "crospi_utils/string_interpolate.hpp"

namespace ros2_control_crospi_utils {

namespace {

const char* PLUGIN_NAME = "ros2_control_crospi";

/** Quotes a string for /bin/sh, so that paths and xacro arguments are passed literally. */
std::string shell_quote(const std::string& s) {
    std::string quoted = "'";
    for (char c : s) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted += c;
        }
    }
    return quoted + "'";
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string run_xacro(const std::string& path, const std::vector<std::pair<std::string, std::string>>& xacro_arguments) {
    std::string command = "xacro " + shell_quote(path);
    for (const auto& [key, value] : xacro_arguments) {
        command += " " + shell_quote(key + ":=" + value);
    }
    command += " 2>&1";   // the output of xacro is shown in the error message if it fails

    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error(fmt::format("[{}] Could not execute '{}'.", PLUGIN_NAME, command));
    }
    std::string output;
    std::array<char, 4096> buffer;
    std::size_t n;
    while ((n = fread(buffer.data(), 1, buffer.size(), pipe)) > 0) {
        output.append(buffer.data(), n);
    }
    const int status = pclose(pipe);
    if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        throw std::runtime_error(fmt::format("[{}] xacro failed (command: {}):\n{}", PLUGIN_NAME, command, output));
    }
    return output;
}

} // namespace


std::string resolve_path(const std::string& path)
{
    std::string p = etasl::string_interpolate(path);  // $[package] and ${ENV}

    const std::string package_prefix = "package://";
    const std::string file_prefix = "file://";
    if (p.rfind(package_prefix, 0) == 0) {
        std::string rest = p.substr(package_prefix.size());
        std::size_t slash = rest.find('/');
        std::string package = rest.substr(0, slash);
        std::string relative = (slash == std::string::npos) ? "" : rest.substr(slash + 1);
        std::string share;
        try {
            share = ament_index_cpp::get_package_share_directory(package);
        } catch (const std::exception& e) {
            throw std::runtime_error(fmt::format("[{}] Could not resolve '{}': package '{}' not found ({}).",
                                                 PLUGIN_NAME, path, package, e.what()));
        }
        p = (std::filesystem::path(share) / relative).string();
    } else if (p.rfind(file_prefix, 0) == 0) {
        p = p.substr(file_prefix.size());
    }
    return std::filesystem::path(p).lexically_normal().string();
}


std::string load_robot_description(const std::string& path,
                                   const std::vector<std::pair<std::string, std::string>>& xacro_arguments)
{
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error(fmt::format("[{}] The robot description '{}' does not exist.", PLUGIN_NAME, path));
    }
    if (ends_with(path, ".xacro")) {
        return run_xacro(path, xacro_arguments);
    }
    if (!xacro_arguments.empty()) {
        throw std::runtime_error(fmt::format("[{}] xacro_arguments are given, but the robot description '{}' is not a "
                                             ".xacro file.", PLUGIN_NAME, path));
    }
    std::ifstream file(path);
    std::stringstream content;
    content << file.rdbuf();
    return content.str();
}

} // namespace ros2_control_crospi_utils
