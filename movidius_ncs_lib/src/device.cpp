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

#include <cassert>
#include <string>

#include <ros/console.h>

#include "movidius_ncs_lib/device.h"
#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/exception_util.h"

namespace movidius_ncs_lib
{
Device::Device(int index, LogLevel log_level)
    : index_(index), handle_(nullptr)
{
  assert(index_ >= 0);
  setLogLevel(log_level);
  find();
  open();
}

Device::~Device()
{
  try
  {
    close();
  }
  catch (MvncException& e)
  {
    ROS_ERROR_STREAM("Exception caught on device[" << index_ << "], " << e.what());
  }
}

std::string Device::getName() const
{
  assert(handle_ != nullptr);
  return name_;
}

void Device::open()
{
  ROS_INFO_STREAM("opening device #" << index_ << " name=" << name_);
  int ret = mvncOpenDevice(name_.c_str(), &handle_);
  ExceptionUtil::tryToThrowMvncException(ret);
}

void Device::close()
{
  if (handle_ == nullptr)
  {
    return;
  }

  ROS_INFO_STREAM("close device #" << index_ << " name=" << name_);
  int ret = mvncCloseDevice(handle_);
  ExceptionUtil::tryToThrowMvncException(ret);
}

void Device::monitorThermal() const
{
  ThermalThrottlingLevel level = getThermalThrottlingLevel();

  if (level == High)
  {
    throw MvncHighThermal();
  }

  if (level == Aggressive)
  {
    throw MvncAggressiveThermal();
  }
}

void Device::setLogLevel(LogLevel log_level)
{
  int level = log_level;
  int ret = mvncSetGlobalOption(MVNC_LOG_LEVEL,
                                static_cast<void*>(&level),
                                sizeof(level));
  ExceptionUtil::tryToThrowMvncException(ret);
}

Device::LogLevel Device::getLogLevel()
{
  int level;
  unsigned int size_of_level = sizeof(level);
  int ret = mvncGetGlobalOption(MVNC_LOG_LEVEL,
                                reinterpret_cast<void**>(&level),
                                &size_of_level);
  ExceptionUtil::tryToThrowMvncException(ret);
  return static_cast<LogLevel>(level);
}

Device::ThermalThrottlingLevel Device::getThermalThrottlingLevel() const
{
  assert(handle_ != nullptr);
  int throttling;
  unsigned int size_of_throttling = sizeof(throttling);
  int ret = mvncGetDeviceOption(handle_,
                                MVNC_THERMAL_THROTTLING_LEVEL,
                                reinterpret_cast<void**>(&throttling),
                                &size_of_throttling);
  ExceptionUtil::tryToThrowMvncException(ret);
  return static_cast<ThermalThrottlingLevel>(throttling);
}

void* Device::getHandle()
{
  assert(handle_ != nullptr);
  return handle_;
}

void Device::find()
{
  assert(handle_ == nullptr);
  char name[MVNC_MAX_NAME_SIZE];
  int ret = mvncGetDeviceName(index_, name, sizeof(name));
  ExceptionUtil::tryToThrowMvncException(ret);
  name_ = name;
}
}  // namespace movidius_ncs_lib

