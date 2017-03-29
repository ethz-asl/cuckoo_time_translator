#ifndef HE5BA64A8_8FA0_4201_BDF7_A01E2976351E
#define HE5BA64A8_8FA0_4201_BDF7_A01E2976351E

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include "MockCuckooClock.h"

namespace ros {
class NodeHandle;
}

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
  MockCuckooClock cuckooClock;
  DeviceTimeUnwrapperAndTranslator<TimestampUnwrapperEventOnly> translator;
};

}

#endif /* HE5BA64A8_8FA0_4201_BDF7_A01E2976351E */
