#include "MockCuckooDeviceDriver.h"

#include <random>

#include <cuckoo_time_translator/KalmanOwt.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <ros/ros.h>
#pragma GCC diagnostic pop

namespace cuckoo_time_translator {
constexpr uint32_t MockCuckooDeviceDriver::kWrappingNumber;
constexpr double MockCuckooDeviceDriver::kFreq;
constexpr double MockCuckooDeviceDriver::kSkew;
constexpr double MockCuckooDeviceDriver::kOffset;

KalmanOwtConfig getMockKalmanConfig() {
  KalmanOwtConfig kc;
  kc.outlierThreshold = 0;
  return kc;
}

MockCuckooDeviceDriver::MockCuckooDeviceDriver(ros::NodeHandle & nh) :
  cuckooClock_(kWrappingNumber, kFreq, kSkew, ros::Duration(kOffset)),
  translator_(WrappingClockParameters{kWrappingNumber, kFreq}, nh.getNamespace(), Defaults().setFilterConfig(getMockKalmanConfig()))
{
  srv_.setCallback(boost::bind(&MockCuckooDeviceDriver::dynamicReconfigureCallback, this, _1, _2));

  nh.param("delay", delaySeconds_, delaySeconds_);
  nh.param("delaySigma", delaySigmaSeconds_, delaySigmaSeconds_);
  nh.param("assumedDelay", assumedDelaySeconds_, assumedDelaySeconds_);
  ROS_INFO("Using initial delay=%g s, delaySigma=%g s, and assumedDelay=%g s.", delaySeconds_, delaySigmaSeconds_, assumedDelaySeconds_);
}


void MockCuckooDeviceDriver::step() {
  ros::Time receiveTime;
  uint32_t deviceTime;

  static std::default_random_engine generator;

  cuckooClock_.getNewSimulatedMeasurementTimes(deviceTime, receiveTime);

  receiveTime += ros::Duration(delaySeconds_);
  if(delaySigmaSeconds_){
    std::exponential_distribution<double> delayDistribution(1/delaySigmaSeconds_);
    receiveTime += ros::Duration(delayDistribution(generator));
  }

  translator_.update(deviceTime, receiveTime, -assumedDelaySeconds_);
}
} // namespace cuckoo_time_translator

using namespace cuckoo_time_translator;




void MockCuckooDeviceDriver::dynamicReconfigureCallback(const MockCuckooClockConfig &config, int) {
  assumedDelaySeconds_ = config.assumedDelay;
  delaySeconds_ = config.delay;
  delaySigmaSeconds_ = config.delaySigma;
  ROS_INFO("Using updated delay=%g s, delaySigma=%g s, and assumedDelay=%g s.", delaySeconds_, delaySigmaSeconds_, assumedDelaySeconds_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mock_cuckoo_device_driver");

  ros::NodeHandle nh;

  ros::Rate loop_rate(1000);

  MockCuckooDeviceDriver mcdd(nh);

  while (ros::ok())
  {
    mcdd.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
