#ifndef HAD8B0C21_4B6C_4EED_92B0_B8653420AD87
#define HAD8B0C21_4B6C_4EED_92B0_B8653420AD87

#include <stdint.h>

#include <ros/common.h>

#include <cuckoo_time_translator/TimestampUnwrapper.h>

namespace cuckoo_time_translator {

class DeviceTimeTranslatorConfig;

enum class FilterAlgorithm {
  None,
  ConvexHull,
  Kalman,
};

class DeviceTimeTranslator {
 public:
  const static std::string kDeviceTimeNamePostfix;

  DeviceTimeTranslator(ros::NodeHandle & nh);
  DeviceTimeTranslator(const std::string & nameSpace, ros::NodeHandle & nh);

  ~DeviceTimeTranslator();

  ros::Time update(const TimestampUnwrapper & eventStamp, const TimestampUnwrapper & transmitStamp, const ros::Time & receiveTime, double offset = 0);

  FilterAlgorithm getCurrentFilterAlgorithm() const;
 private:
  void configCallback(DeviceTimeTranslatorConfig &config, uint32_t level);

  class Impl;
  Impl *pImpl_;
};

class DeviceTimeUnwrapperAndTranslator {
 public:
  DeviceTimeUnwrapperAndTranslator(TimestampUnwrapper timestampUnwrapper, ros::NodeHandle & nh);
  DeviceTimeUnwrapperAndTranslator(TimestampUnwrapper timestampUnwrapper, const std::string & nodeName, ros::NodeHandle & nh);
  ~DeviceTimeUnwrapperAndTranslator();

  ros::Time update(uint32_t eventStamp, uint32_t transmitStamp, const ros::Time & receiveTime, double offset = 0.0);
  ros::Time update(uint32_t eventStamp, const ros::Time & receiveTime, double offset = 0.0);

 private:
  TimestampUnwrapper eventUnwrapper, transmitUnwrapper;
  DeviceTimeTranslator translator;
};


}

#endif /* HAD8B0C21_4B6C_4EED_92B0_B8653420AD87 */
