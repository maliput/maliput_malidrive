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
#include "utility/resources.h"

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace utility {
namespace {

const char* kMalidriveEnvVariable{"MALIPUT_MALIDRIVE_RESOURCE_ROOT"};

// Retrieves a list of paths that live in `env_path` env variable.
std::vector<std::string> GetAllPathDirectories(const std::string& env_path) {
  std::istringstream path_stream(std::string(std::getenv(env_path.c_str())));
  const std::string delimeter{":"};
  std::vector<std::string> paths;
  std::string path;
  while (std::getline(path_stream, path, ':')) {
    paths.push_back(path);
  }
  return paths;
}

// Appends `relative_path` to the end of `base_path` adding the path delimiter
// ('/').
std::string AppendPath(const std::string& base_path, const std::string& relative_path) {
  const std::string path_delimeter("/");
  if (base_path.back() != *(path_delimeter.c_str())) return base_path + path_delimeter + relative_path;
  return base_path + relative_path;
}

}  // namespace

std::string FindResourceInPath(const std::string& resource_name, const std::string& path_to_resources) {
  const std::string file_path = AppendPath(path_to_resources, resource_name);
  if (std::ifstream(file_path)) {
    return file_path;
  }
  throw std::runtime_error(std::string("Resource: ") + resource_name + std::string(" couldn't be found."));
}

std::string FindResourceInEnvPath(const std::string& resource_name, const std::string& path_to_resources) {
  const std::vector<std::string> env_paths = GetAllPathDirectories(path_to_resources);
  for (const std::string& env_path : env_paths) {
    const std::string file_path = AppendPath(env_path, resource_name);
    if (std::ifstream(file_path)) {
      return file_path;
    }
  }
  throw std::runtime_error(std::string("Resource: ") + resource_name + std::string(" couldn't be found."));
}

std::string FindResource(const std::string& resource_name) {
  const std::string resources_folder{"resources"};
  return FindResourceInEnvPath(AppendPath(resources_folder, resource_name), kMalidriveEnvVariable);
}

}  // namespace utility
