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