#ifndef HAD8B0C21_4B6C_4EED_92B0_B8653420AD87
#define HAD8B0C21_4B6C_4EED_92B0_B8653420AD87

#include <stdint.h>

#include <ros/common.h>

#include <cuckoo_time_translator/TimestampUnwrapper.h>

namespace cuckoo_time_translator {

class DeviceTimeTranslatorConfig;

struct FilterAlgorithm {
  enum Type {
    None,
    ConvexHull,
    Kalman,
  } type;

  FilterAlgorithm(Type t) : type(t) {}
  bool operator == (const FilterAlgorithm & other) { return type == other.type; }
  bool operator != (const FilterAlgorithm & other) { return type != other.type; }
};


class DeviceTimeTranslator {
 public:
  const static std::string kDeviceTimeNamePostfix;

  DeviceTimeTranslator(ros::NodeHandle & nh);
  DeviceTimeTranslator(const std::string & nameSpace, ros::NodeHandle & nh);

  ~DeviceTimeTranslator();

  ros::Time update(const TimestampUnwrapper & eventStamp, const TimestampUnwrapper & transmitStamp, const ros::Time & receiveTime, double offset = 0);
  ros::Time translate(const TimestampUnwrapper & eventStampUnwrapper, UnwrappedStamp unwrappedEventStamp) const;

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

  /**
   * Unwrap a new event's device time stamp.
   *
   * All calls to the update functions (above) and this function _must_ precisely honor the strict order of the events themselves.
   * Actually the whole method family must not be called twice for the same event or two events that have the same device timestamp.
   * Otherwise this will lead to wrongly assumed wraps of the device clock and therefore sudden acceleration of device time!
   *
   * @param eventStamp the event's stamp assigned by the device.
   * @return the translated local time
   */
  UnwrappedStamp unwrapEventStamp(uint32_t eventStamp);

  ros::Time translate(UnwrappedStamp unwrappedStamp) const;
 private:
  TimestampUnwrapper eventUnwrapper, transmitUnwrapper;
  DeviceTimeTranslator translator;
};


}

#endif /* HAD8B0C21_4B6C_4EED_92B0_B8653420AD87 */
