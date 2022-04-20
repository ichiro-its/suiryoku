// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <fstream>
#include <iomanip>
#include <string>

#include "suiryoku/config/utils/config.hpp"

#include "nlohmann/json.hpp"

namespace suiryoku
{

Config::Config(const std::string & path)
: path(path)
{
}

std::string Config::get_config() const
{
  std::ifstream file(path + "locomotion.json");
  nlohmann::json data = nlohmann::json::parse(file);

  file.close();
  return data.dump();
}

void Config::set_config(const nlohmann::json & data)
{
  std::ofstream file(path + "locomotion.json", std::ios::out | std::ios::trunc);
  file << std::setw(2) << data << std::endl;
  file.close();
}

}  // namespace suiryoku
