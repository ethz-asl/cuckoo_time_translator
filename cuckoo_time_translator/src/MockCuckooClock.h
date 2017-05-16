#ifndef HA5447364_688E_4FF7_85A5_F1840F5D245F
#define HA5447364_688E_4FF7_85A5_F1840F5D245F

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "ros/time.h"
#pragma GCC diagnostic pop

namespace cuckoo_time_translator {

class MockCuckooClock {
 public:
  MockCuckooClock(uint64_t wrapAroundNumber, double clockFrequency, double skew, ros::Duration offset);
  virtual ~MockCuckooClock();
  void getNewSimulatedMeasurementTimes(uint32_t & deviceTime, ros::Time & receiveTime);
 private:
  double clockFrequency_;
  uint64_t wrapAroundNumber_;

  ros::Time startTime_;
  double skew_;
  uint32_t lastSimulatedDeviceTime_;
};

} /* namespace cuckoo_time_translator */

#endif /* HA5447364_688E_4FF7_85A5_F1840F5D245F */
