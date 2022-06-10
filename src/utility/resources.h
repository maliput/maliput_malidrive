// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <optional>
#include <string>

namespace utility {

// Finds a resource file by its `resource_name` name.
//
// The first valid file within path_directory/`resource_name` is returned.
// Where path_directory is indicated by MALIPUT_MALIDRIVE_RESOURCE_ROOT env variable.
// Path delimeter is assumed to be `/`.
// @param resource_name The name of the file.
// @return The complete file path to the resource.
// @throws std::runtime_error When the resource cannot be found under any
// of the possible combinations of 'path_directory/resource_name'.
std::string FindResource(const std::string& resource_name);

// Finds a resource file by its `resource_name` name.
//
// The first valid `resource_name` file within the paths pointed by the environment variable `path_to_resources` is
// returned. Path delimeter is assumed to be `/`.
//
// @param resource_name The name of the file.
// @param path_to_resources Env variable holding one or many directory path.
// @return The complete file path to the resource.
// @throws std::runtime_error When the resource cannot be found for any path.
std::string FindResourceInEnvPath(const std::string& resource_name, const std::string& path_to_resources);

// Finds a resource file by its `resource_name` name.
//
// The first valid `resource_name` file within `path_to_resources`.
//
// @param resource_name The name of the file.
// @param path_to_resources Env variable holding one or many directory path.
// @return The complete file path to the resource.
// @throws std::runtime_error When the resource cannot be found for any path.
std::string FindResourceInPath(const std::string& resource_name, const std::string& path_to_resources);

}  // namespace utility
