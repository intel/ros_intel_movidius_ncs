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

#ifndef MOVIDIUS_NCS_LIB_DEVICE_H
#define MOVIDIUS_NCS_LIB_DEVICE_H

#include <string>
#include <memory>
#include <movidius_ncs_lib/mvnc_cpp.h>

namespace movidius_ncs_lib
{
class Device
{
public:
  using Ptr = std::shared_ptr<Device>;
  using ConstPtr = std::shared_ptr<Device const>;

  enum LogLevel
  {
    Nothing,
    Errors,
    Verbose
  };

  Device(int index, LogLevel log_level);
  ~Device();

  void setLogLevel(LogLevel level);
  LogLevel getLogLevel();
  void* getHandle();
  std::string getName() const;
  void monitorThermal() const;

private:
  enum ThermalThrottlingLevel
  {
    Normal,
    High,
    Aggressive
  };

  void open();
  void close();

  void find();
  ThermalThrottlingLevel  getThermalThrottlingLevel() const;

  int index_;
  std::string name_;
  void* handle_;
};
}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_DEVICE_H
