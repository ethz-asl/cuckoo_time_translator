#ifndef HE5BA64A8_8FA0_4201_BDF7_A01E2976351E
#define HE5BA64A8_8FA0_4201_BDF7_A01E2976351E

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include "MockCuckooClock.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <dynamic_reconfigure/server.h>
#include <cuckoo_time_translator/MockCuckooClockConfig.h>
#pragma GCC diagnostic pop

namespace cuckoo_time_translator {

class MockCuckooDeviceDriver {
 public:
  constexpr static uint32_t kWrappingNumber = 10000000ul;
  constexpr static double kFreq = 1e6;
  constexpr static double kSkew = 3.0;
  constexpr static double kOffset = -5.0;

  MockCuckooDeviceDriver(ros::NodeHandle & nh);

  void step();
 private:
  void dynamicReconfigureCallback(const MockCuckooClockConfig &config, int);

  MockCuckooClock cuckooClock_;
  DeviceTimeUnwrapperAndTranslator<TimestampUnwrapperEventOnly> translator_;
  double delaySeconds_ = 0;
  double delaySigmaSeconds_ = 0;
  double assumedDelaySeconds_ = 0;

  dynamic_reconfigure::Server<MockCuckooClockConfig> srv_;
};

}

#endif /* HE5BA64A8_8FA0_4201_BDF7_A01E2976351E */
