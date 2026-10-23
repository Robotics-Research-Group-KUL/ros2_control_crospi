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

#pragma once

#include <string>
#include <utility>
#include <vector>

namespace ros2_control_crospi_utils {

/**
 * @brief Resolves a path that may contain $[package] / ${ENV} (crospi string interpolation), package://package/...
 *        or file:// prefixes to an absolute path.
 * @throws std::runtime_error if a package cannot be found.
 */
std::string resolve_path(const std::string& path);

/**
 * @brief Returns the robot description (URDF string) of the file at path. Files ending with .xacro are processed with
 *        the xacro executable using the given name:=value arguments; other files are read as they are.
 * @throws std::runtime_error if the file does not exist, if xacro fails, or if arguments are given for a non-xacro file.
 */
std::string load_robot_description(const std::string& path,
                                   const std::vector<std::pair<std::string, std::string>>& xacro_arguments);

} // namespace ros2_control_crospi_utils
