#include "MockCuckooClock.h"

#include <chrono>
#include <thread>

namespace cuckoo_time_translator {

MockCuckooClock::MockCuckooClock(uint64_t wrapAroundNumber, double clockFrequency, double skew, ros::Duration offset) :
    clockFrequency_(clockFrequency),
    wrapAroundNumber_(wrapAroundNumber),
    startTime_(ros::Time::now() + offset),
    skew_(skew),
    lastSimulatedDeviceTime_(0u)
{
}

MockCuckooClock::~MockCuckooClock() {
}

void MockCuckooClock::getNewSimulatedMeasurementTimes(uint32_t& deviceTime, ros::Time& receiveTime) {
  do {
    auto now = ros::Time::now();
    auto delta = now - startTime_;
    uint32_t simulatedDeviceTime = uint32_t(double(delta.toNSec()) * 1e-9 * clockFrequency_ / skew_) % wrapAroundNumber_;
    if(simulatedDeviceTime != lastSimulatedDeviceTime_){ // make sure we get a different device time each time
      lastSimulatedDeviceTime_ = simulatedDeviceTime;
      deviceTime = simulatedDeviceTime;
      receiveTime = now;
      return;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  } while(true);
}

} /* namespace cuckoo_time_translator */
