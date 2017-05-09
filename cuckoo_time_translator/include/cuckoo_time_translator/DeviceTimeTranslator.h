#ifndef HAD8B0C21_4B6C_4EED_92B0_B8653420AD87
#define HAD8B0C21_4B6C_4EED_92B0_B8653420AD87

#include <stdint.h>

#include <ros/common.h>

#include <cuckoo_time_translator/TimestampUnwrapper.h>

namespace cuckoo_time_translator {

class DeviceTimeTranslatorConfig;

//TODO (c++11) use enum class
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

  DeviceTimeTranslator(const std::string & nameSpace);

  ~DeviceTimeTranslator();

  ros::Time update(const TimestampUnwrapper & timestampUnwrapper, const ros::Time & receiveTime, double offset = 0);
  ros::Time translate(const TimestampUnwrapper & timestampUnwrapper, UnwrappedStamp unwrappedEventStamp) const;

  FilterAlgorithm getCurrentFilterAlgorithm() const;
  void setFilterAlgorithm(FilterAlgorithm filterAlgorithm);

 private:
  void configCallback(DeviceTimeTranslatorConfig &config, uint32_t level);

  class Impl;
  Impl *pImpl_;
};

template <typename Unwrapper_ = TimestampUnwrapperEventOnly>
class DeviceTimeUnwrapperAndTranslator {
 public:
  typedef Unwrapper_ Unwrapper;
  typedef typename Unwrapper::Timestamp Timestamp;
  typedef typename Unwrapper_::UnwrapperClockParameters UnwrapperClockParameters;

  DeviceTimeUnwrapperAndTranslator(const UnwrapperClockParameters & clockParameters, const std::string & nameSpace);

  ros::Time update(Timestamp eventStamp, const ros::Time & receiveTime, double offset = 0.0);

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
  UnwrappedStamp unwrapEventStamp(Timestamp eventStamp);

  ros::Time translate(UnwrappedStamp unwrappedStamp) const;

  FilterAlgorithm getCurrentFilterAlgorithm() const {
    return translator.getCurrentFilterAlgorithm();
  }
  void setFilterAlgorithm(FilterAlgorithm filterAlgorithm) {
    translator.setFilterAlgorithm(filterAlgorithm);
  }
 protected:
  Unwrapper timestampUnwrapper;
  DeviceTimeTranslator translator;
};

template <typename Unwrapper_ = TimestampUnwrapperEventAndTransmit>
class DeviceTimeUnwrapperAndTranslatorWithTransmitTime : public DeviceTimeUnwrapperAndTranslator<Unwrapper_> {
 public:
  typedef Unwrapper_ Unwrapper;
  typedef typename Unwrapper_::TransmitTimestamp Timestamp;
  typedef typename Unwrapper_::UnwrapperClockParameters UnwrapperClockParameters;

  //TODO (c++11) inherit constructor
  DeviceTimeUnwrapperAndTranslatorWithTransmitTime(const UnwrapperClockParameters & clockParameters, const std::string & nameSpace);

  ros::Time update(Timestamp eventStamp, Timestamp transmitStamp, const ros::Time & receiveTime, double offset = 0.0);
  UnwrappedStamp unwrapTransmitStamp(Timestamp eventStamp);
};

class DefaultDeviceTimeUnwrapperAndTranslator : public DeviceTimeUnwrapperAndTranslator<> {
 public:
  DefaultDeviceTimeUnwrapperAndTranslator(const WrappingClockParameters & wrappingClockParameters, const std::string & nameSpace) :
    DeviceTimeUnwrapperAndTranslator<>(wrappingClockParameters, nameSpace) {}
};

class UnwrappedDeviceTimeTranslator : public DeviceTimeUnwrapperAndTranslator<TimestampPassThrough> {
 public:
  UnwrappedDeviceTimeTranslator(ClockParameters clockParameters, const std::string & nameSpace) :
    DeviceTimeUnwrapperAndTranslator<TimestampPassThrough>(clockParameters, nameSpace) {}

  ros::Time translate(TimestampPassThrough::Timestamp timestamp) const {
    return DeviceTimeUnwrapperAndTranslator<TimestampPassThrough>::translate(this->timestampUnwrapper.toUnwrapped(timestamp));
  }

};

class DefaultDeviceTimeUnwrapperAndTranslatorWithTransmitTime : public DeviceTimeUnwrapperAndTranslatorWithTransmitTime<> {
 public:
  DefaultDeviceTimeUnwrapperAndTranslatorWithTransmitTime(const WrappingClockParameters & wrappingClockParameters, const std::string & nameSpace) :
    DeviceTimeUnwrapperAndTranslatorWithTransmitTime<>(wrappingClockParameters, nameSpace) {}
};

}

#endif /* HAD8B0C21_4B6C_4EED_92B0_B8653420AD87 */
