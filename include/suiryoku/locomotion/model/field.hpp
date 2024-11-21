#ifndef SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_
#define SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_

#include <vector>

#include "keisan/keisan.hpp"

namespace suiryoku
{

class Field
{
public:
  Field();

  const double width;
  const double length;
  const double num_landmarks;
  std::vector<keisan::Point2> landmarks;
};

}  // namespace suiryoku

#endif  // SUIRYOKU__LOCOMOTION__MODEL__FIELD_HPP_