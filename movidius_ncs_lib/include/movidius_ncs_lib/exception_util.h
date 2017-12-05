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

#ifndef MOVIDIUS_NCS_LIB_EXCEPTION_UTIL_H
#define MOVIDIUS_NCS_LIB_EXCEPTION_UTIL_H

#include <map>

namespace movidius_ncs_lib
{
class ExceptionUtil
{
public:
  static void tryToThrowMvncException(int code);
};
}   // namespace movidius_ncs_lib
#endif  // MOVIDIUS_NCS_LIB_EXCEPTION_UTIL_H

