/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MOVIDIUS_NCS_LIB_EXCEPTION_H
#define MOVIDIUS_NCS_LIB_EXCEPTION_H

#include <exception>
#include <string>

namespace movidius_ncs_lib
{
struct NCSException: public std::exception {};

struct MvncException: public NCSException
{
  explicit MvncException(const std::string& msg)
      : msg_(msg)
  {
  }

  const char* what() const noexcept
  {
    return msg_.c_str();
  }

private:
  const std::string msg_;
};

struct MvncBusy: public MvncException
{
  explicit MvncBusy(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncError: public MvncException
{
  explicit MvncError(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncOutOfMemory: public MvncException
{
  explicit MvncOutOfMemory(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncDeviceNotFound: public MvncException
{
  explicit MvncDeviceNotFound(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncInvalidParameters: public MvncException
{
  explicit MvncInvalidParameters(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncTimeout: public MvncException
{
  explicit MvncTimeout(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncMvCmdNotFound: public MvncException
{
  explicit MvncMvCmdNotFound(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncNoData: public MvncException
{
  explicit MvncNoData(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncGone: public MvncException
{
  explicit MvncGone(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncUnsupportedGraphFile: public MvncException
{
  explicit MvncUnsupportedGraphFile(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncMyriadError: public MvncException
{
  explicit MvncMyriadError(const std::string& msg)
    : MvncException(msg)
  {
  }
};

struct MvncThermalException: public NCSException
{
  virtual const char* what() const noexcept = 0;
};

struct MvncHighThermal: public MvncThermalException
{
  const char* what() const noexcept;
};

struct MvncAggressiveThermal: public MvncThermalException
{
  const char* what() const noexcept;
};

struct NCSGraphException: public NCSException
{
  virtual const char* what() const noexcept = 0;
};

struct NCSGraphFileError: public NCSGraphException
{
  const char* what() const noexcept;
};

struct NCSMeanAndStddevError: public NCSGraphException
{
  const char* what() const noexcept;
};

struct NCSInputSizeFileError: public NCSGraphException
{
  const char* what() const noexcept;
};

struct NCSInputSizeError: public NCSGraphException
{
  const char* what() const noexcept;
};

struct NCSLoadCategoriesError: public NCSGraphException
{
  const char* what() const noexcept;
};
}  // namespace movidius_ncs_lib
#endif  // MOVIDIUS_NCS_LIB_EXCEPTION_H
