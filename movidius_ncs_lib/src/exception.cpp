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

#include <movidius_ncs_lib/exception.h>

namespace movidius_ncs_lib
{

const char* MvncHighThermal::what() const noexcept
{
  return "lower guard temperature threshold is reached, short throtting is in action";
}

const char* MvncAggressiveThermal::what() const noexcept
{
  return "upper guard temperature threshold is reached, long throtting is in action";
}

const char* NCSGraphFileError::what() const noexcept
{
  return "cannot load graph";
}

const char* NCSMeanAndStddevError::what() const noexcept
{
  return "cannot load mean and stddev";
}

const char* NCSInputSizeFileError::what() const noexcept
{
  return "cannot open inputsize file";
}

const char* NCSInputSizeError::what() const noexcept
{
  return "cannot load inputsize";
}

const char* NCSLoadCategoriesError::what() const noexcept
{
  return "cannot load categories";
}
}  // namespace movidius_ncs_lib
