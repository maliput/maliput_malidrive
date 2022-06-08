// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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

#include <string>

#include <maliput/common/assertion_error.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/common/maliput_throw.h>

/// @def MALIDRIVE_DEMAND
/// Used to declare a demand. Will quit execution otherwise.
#define MALIDRIVE_DEMAND(condition) MALIPUT_DEMAND(condition)

/// @def MALIDRIVE_DEMAND
/// Aborts the program (via \::abort) with a message showing at least the
/// given message (macro argument), function name, file, and line.
#define MALIDRIVE_ABORT_MSG(msg) MALIPUT_ABORT_MESSAGE(msg)

/// @def MALIDRIVE_IS_IN_RANGE
/// Throws if `value` is within [`min_value`; `max_value`]. It forwards the call
/// to MALIDRIVE_VALIDATE() with a customized string stating the error.
#define MALIDRIVE_IS_IN_RANGE(value, min_value, max_value)                                       \
  do {                                                                                           \
    MALIDRIVE_VALIDATE(value >= min_value, maliput::common::assertion_error,                     \
                       std::to_string(value) + " is less than " + std::to_string(min_value));    \
    MALIDRIVE_VALIDATE(value <= max_value, maliput::common::assertion_error,                     \
                       std::to_string(value) + " is greater than " + std::to_string(max_value)); \
  } while (0)

/// @def MALIDRIVE_THROW_UNLESS
/// Used to declare a conditional throw.
#define MALIDRIVE_THROW_UNLESS(condition) MALIPUT_THROW_UNLESS(condition)

/// @def MALIDRIVE_THROW_MESSAGE
/// Throws with `msg`.
#define MALIDRIVE_THROW_MESSAGE(msg)                                              \
  do {                                                                            \
    const std::string message(msg);                                               \
    MALIDRIVE_VALIDATE(false, maliput::common::assertion_error, message.c_str()); \
  } while (0)

/// @def MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN
/// Deletes the special member functions for copy-construction, copy-assignment,
/// move-construction, and move-assignment.
/// Invoke this macro in the public section of the class declaration, e.g.:
/// <pre>
/// class Foo {
///  public:
///   MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)
///
///   // ...
/// };
/// </pre>
/// */
#define MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Classname) MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)

/// @def MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN
/// Implements default special member functions for copy-construction,
/// copy-assignment, move-construction, and move-assignment.
/// Invoke this macro in the public section of the class declaration, e.g.:
/// <pre>
/// class Foo {
///  public:
///   MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)
///
///   // ...
/// };
/// </pre>
/// */
#define MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname) MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)

/// @def STR_SIMPLE
/// Internal stringify a token
#define STR_SIMPLE(x) #x

/// @def STR
/// Stringify a token
#define STR(x) STR_SIMPLE(x)

/// @def MALIDRIVE_VALIDATE
/// Used to validate that an argument passed into a function or method is true;
/// if not, an exception of type exctype is thrown.
#define MALIDRIVE_VALIDATE(pred, exctype, message)                           \
  do {                                                                       \
    if (!(pred)) {                                                           \
      std::string fullname = std::string(__FILE__);                          \
      size_t found = fullname.find_last_of("/");                             \
      std::string fname = fullname;                                          \
      if (found != std::string::npos) {                                      \
        fname = fullname.substr(found + 1);                                  \
      }                                                                      \
      std::string errmsg(fname);                                             \
      errmsg.append(":").append(__func__).append(":").append(STR(__LINE__)); \
      errmsg.append(": ").append(message);                                   \
      throw exctype(errmsg);                                                 \
    }                                                                        \
  } while (0)

/// @def MALIDRIVE_TRACE
/// Rely on maliput::log->trace(). Used to trace log `msg` and add
/// information of file, line and function.
#define MALIDRIVE_TRACE(msg)                                                           \
  do {                                                                                 \
    const std::string message(msg);                                                    \
    maliput::log()->trace("[\"{}\":{}:{}] {}", __FILE__, __LINE__, __func__, message); \
  } while (0)
