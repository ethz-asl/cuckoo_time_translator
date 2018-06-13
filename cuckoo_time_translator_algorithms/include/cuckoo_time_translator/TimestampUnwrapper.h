#ifndef HDA9FE6E4_D75C_4B1A_896A_98CF800AB084
#define HDA9FE6E4_D75C_4B1A_896A_98CF800AB084

#include <stdint.h>

#include "AbstractAssert.h"
#include "ClockParameters.h"

// TODO (c++11) remove this
#if __cplusplus <= 199711L
#define OVERRIDE
#else
#define OVERRIDE override
#endif


namespace cuckoo_time_translator {

class TimestampUnwrapper {
 public:
  virtual ~TimestampUnwrapper();

  virtual UnwrappedStamp getUnwrappedEventStamp() const = 0;
  virtual UnwrappedStamp getUnwrappedTransmitStamp() const = 0;

  virtual double stampToSec(UnwrappedStamp stamp) const = 0;

  double getEventStampSec() const {
    return stampToSec(getUnwrappedEventStamp());
  }
  double getTransmitStampSec() const {
    return stampToSec(getUnwrappedTransmitStamp());
  }

  virtual double getClockFrequencyHz() const = 0;

  virtual bool hasSeparateTransmitTime() const = 0;
};

class TimestampPassThrough : public TimestampUnwrapper {
 public:
  typedef uint64_t Timestamp;
  typedef ClockParameters UnwrapperClockParameters;

  TimestampPassThrough(const UnwrapperClockParameters & clockParameters);
  virtual ~TimestampPassThrough();

  void updateWithNewEventStamp(Timestamp newDeviceStamp);
  UnwrappedStamp toUnwrapped(Timestamp deviceStamp) const { return deviceStamp; }

  virtual UnwrappedStamp getUnwrappedEventStamp() const OVERRIDE;
  virtual UnwrappedStamp getUnwrappedTransmitStamp() const OVERRIDE;

  virtual double stampToSec(UnwrappedStamp stamp) const OVERRIDE;

  virtual double getClockFrequencyHz() const OVERRIDE;

  virtual bool hasSeparateTransmitTime() const OVERRIDE;
 private:
  ClockParameters clockParams_;
  UnwrappedStamp lastDeviceStamp_;
};

namespace internal {

class Unwrapper {
 public:
  Unwrapper();

  void updateWithNewStamp(WrappingClockParameters & wrappingClockParameters, uint64_t newDeviceStamp);

  UnwrappedStamp getUnwrappedStamp(const WrappingClockParameters & wrappingClockParameters) const {
    return UnwrappedStamp(wrapsCounter_ * wrappingClockParameters.getWrapAroundNumber() + lastStamp_);
  }
 private:
  uint64_t wrapsCounter_, lastStamp_;
};

class AbstractTimestampUnwrapper : public TimestampUnwrapper {
 public:
  typedef WrappingClockParameters UnwrapperClockParameters;

  AbstractTimestampUnwrapper(const UnwrapperClockParameters & wrappingClockParameters);

  virtual double stampToSec(UnwrappedStamp stamp) const OVERRIDE;

  double getClockFrequencyHz() const OVERRIDE {
    return clockParams_.getClockFrequencyHz();
  }

  void setClockFrequencyHz(double clockFrequencyHz) {
    clockParams_.setClockFrequencyHz(clockFrequencyHz);
  }

  uint64_t getWrapAroundNumber() const {
    return clockParams_.getWrapAroundNumber();
  }

 protected:
  WrappingClockParameters clockParams_;
};

}

class TimestampUnwrapperEventOnly : public internal::AbstractTimestampUnwrapper {
 public:
  typedef uint64_t Timestamp;

  TimestampUnwrapperEventOnly(const WrappingClockParameters & wrappingClockParameters);

  virtual UnwrappedStamp getUnwrappedEventStamp() const OVERRIDE;
  virtual UnwrappedStamp getUnwrappedTransmitStamp() const OVERRIDE;

  void updateWithNewEventStamp(Timestamp newDeviceStamp);

  virtual bool hasSeparateTransmitTime() const OVERRIDE;
 protected:
  internal::Unwrapper eventUnwrapper_;
};

class TimestampUnwrapperEventAndTransmit : public internal::AbstractTimestampUnwrapper {
 public:
  typedef uint64_t Timestamp;
  typedef Timestamp TransmitTimestamp;

  TimestampUnwrapperEventAndTransmit(const WrappingClockParameters & wrappingClockParameters);

  virtual UnwrappedStamp getUnwrappedEventStamp() const OVERRIDE;
  virtual UnwrappedStamp getUnwrappedTransmitStamp() const OVERRIDE;

  void updateWithNewEventStamp(Timestamp newDeviceStamp);
  void updateWithNewTransmitStamp(Timestamp newDeviceStamp);

  bool hasSeparateTransmitTime() const OVERRIDE;

 private:
  internal::Unwrapper eventUnwrapper_;
  internal::Unwrapper transmitUnwrapper_;
};

#undef OVERRIDE

} // namespace cuckoo_time_translator


#endif /* HDA9FE6E4_D75C_4B1A_896A_98CF800AB084 */
