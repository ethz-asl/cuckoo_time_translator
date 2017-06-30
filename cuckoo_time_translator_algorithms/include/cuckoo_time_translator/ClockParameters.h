#ifndef H19477E2D_1466_4810_AB77_E1F3D818A1E3
#define H19477E2D_1466_4810_AB77_E1F3D818A1E3

#include <stdint.h>

namespace cuckoo_time_translator {

namespace internal {
  class Unwrapper;
}

class UnwrappedStamp {
 public:
  uint64_t getValue() const {
    return stamp_;
  }
 private:
  UnwrappedStamp(uint64_t stamp) : stamp_(stamp){}
  friend class internal::Unwrapper;
  friend class TimestampPassThrough;
  uint64_t stamp_;
};

class ClockParameters {
 public:
  ClockParameters(double clockFrequencyHz);

  double stampToSec(UnwrappedStamp stamp) const {
    return (double) stamp.getValue() / clockFrequencyHz_;
  }

  double getClockFrequencyHz() const {
    return clockFrequencyHz_;
  }

  void setClockFrequencyHz(double clockFrequencyHz) {
    clockFrequencyHz_ = clockFrequencyHz;
  }
 private:
  double clockFrequencyHz_;
};

class WrappingClockParameters : public ClockParameters {
 public:
  WrappingClockParameters(uint64_t wrapAroundNumber, double clockFrequencyHz);
  virtual ~WrappingClockParameters();

  uint64_t getWrapAroundNumber() const {
    return wrapAroundNumber_;
  }
 private:
  void checkNewDeviceStamp(uint64_t wrapsCounter, uint64_t newDeviceStamp);

  friend class internal::Unwrapper;
  uint64_t maxStamp_;
  uint64_t wrapAroundNumber_;
};

} // namespace cuckoo_time_translator

#endif /* H19477E2D_1466_4810_AB77_E1F3D818A1E3 */
