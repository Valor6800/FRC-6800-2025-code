#include <hal/HAL.h>

#include "gtest/gtest.h"
#include "valkyrie/sensors/GamePieceSensor.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
