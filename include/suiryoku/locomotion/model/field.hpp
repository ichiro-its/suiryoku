// Copyright (c) 2025 Ichiro ITS
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

#ifndef SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_
#define SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_

#include <vector>

#include "keisan/keisan.hpp"

namespace suiryoku
{

struct Field
{
public:
  int width;
  int length;
  std::vector<keisan::Point2> landmarks_L;
  std::vector<keisan::Point2> landmarks_T;
  std::vector<keisan::Point2> landmarks_X;
  std::vector<keisan::Point2> landmarks_goalpost;

  Field()
  : width(600),
    length(900),
    landmarks_L({
      {0.0, 0.0},
      {0.0, 600.0},
      {100.0, 50.0},
      {100.0, 550.0},
      {900.0, 0.0},
      {900.0, 600.0},
      {800.0, 50.0},
      {800.0, 550.0}
    }),
    landmarks_T({
      {0.0, 50.0},
      {0.0, 550.0},
      {450.0, 0.0},
      {450.0, 600.0},
      {900.0, 50.0},
      {900.0, 550.0}
    }),
    landmarks_X({
      {210, 300},
      {690, 300},
      {450, 300},
      {450, 375},
      {450, 225}
    }),
    landmarks_goalpost({
      {0.0, 170.0},
      {0.0, 430.0},
      {900.0, 170.0},
      {900.0, 430.0}
    })
  {
  }
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_
