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

#include <map>
#include <string>
#include <cassert>
#include <movidius_ncs_lib/exception.h>
#include <movidius_ncs_lib/mvnc_cpp.h>
#include "movidius_ncs_lib/exception_util.h"

namespace movidius_ncs_lib
{
std::map<int, std::string> CODE2STR =
{
  {
    MVNC_BUSY,
    "device is busy, retry later"
  }
  ,
  {
    MVNC_ERROR,
    "an unexpected error was encontered during the function call"
  }
  ,
  {
    MVNC_OUT_OF_MEMORY,
    "the host is out of memory"
  }
  ,
  {
    MVNC_DEVICE_NOT_FOUND,
    "there is no device at the given index or name"
  }
  ,
  {
    MVNC_INVALID_PARAMETERS,
    "at least one of the given parameters is invalid in the context of the function call"
  }
  ,
  {
    MVNC_TIMEOUT,
    "timeout in the communication with the device"
  }
  ,
  {
    MVNC_MVCMDNOTFOUND,
    "the file named MvNCAPI.mvcmd should be installed in the mvnc direcotry"
  }
  ,
  {
    MVNC_NODATA,
    "no data to return"
  }
  ,
  {
    MVNC_GONE,
    "the graph or device has been closed during the operation"
  }
  ,
  {
    MVNC_UNSUPPORTEDGRAPHFILE,
    "the graph file may have been created with an incompatible prior version of the Toolkit"
  }
  ,
  {
    MVNC_MYRIADERROR,
    "an error has been reported by Movidius VPU"
  }
};

void ExceptionUtil::tryToThrowMvncException(int code)
{
  assert(MVNC_OK >= code && code >= MVNC_MYRIADERROR);

  try
  {
    const std::string msg = CODE2STR.at(code);

    if (code == MVNC_BUSY)
    {
      throw MvncBusy(msg);
    }

    if (code == MVNC_ERROR)
    {
      throw MvncError(msg);
    }

    if (code == MVNC_OUT_OF_MEMORY)
    {
      throw MvncOutOfMemory(msg);
    }

    if (code == MVNC_INVALID_PARAMETERS)
    {
      throw MvncInvalidParameters(msg);
    }

    if (code == MVNC_TIMEOUT)
    {
      throw MvncTimeout(msg);
    }

    if (code == MVNC_MVCMDNOTFOUND)
    {
      throw MvncMvCmdNotFound(msg);
    }

    if (code == MVNC_NODATA)
    {
      throw MvncNoData(msg);
    }

    if (code == MVNC_GONE)
    {
      throw MvncGone(msg);
    }

    if (code == MVNC_UNSUPPORTEDGRAPHFILE)
    {
      throw MvncUnsupportedGraphFile(msg);
    }

    if (code == MVNC_MYRIADERROR)
    {
      throw MvncMyriadError(msg);
    }
  }
  catch (std::out_of_range& e)
  {
  }
}
}   // namespace movidius_ncs_lib
