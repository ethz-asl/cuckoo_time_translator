#ifndef HDA9FE6E4_D75C_4B1A_896A_98CF800AB084
#define HDA9FE6E4_D75C_4B1A_896A_98CF800AB084

#include <stdint.h>

namespace cuckoo_time_translator {
class TimestampUnwrapper {
 public:
  TimestampUnwrapper(uint64_t wrapAroundNumber, double clockFrequencyHz);

  void updateWithNewStamp(uint32_t newHwStamp);

  uint64_t getUnwrappedStamp() const {
    return (uint64_t) wrapsCounter_ * wrapAroundNumber_ + lastStamp_;
  }

  double toSec() const {
    return (double) getUnwrappedStamp() / clockFrequencyHz_;
  }

  double getClockFrequencyHz() const {
    return clockFrequencyHz_;
  }

  void setClockFrequencyHz(double clockFrequencyHz) {
    clockFrequencyHz_ = clockFrequencyHz;
  }

  uint64_t getWrapAroundNumber() const {
    return wrapAroundNumber_;
  }

 private:
  uint32_t wrapsCounter_, lastStamp_, maxStamp_;
  uint64_t wrapAroundNumber_;
  double clockFrequencyHz_;
};

} // namespace cuckoo_time_translator


#endif /* HDA9FE6E4_D75C_4B1A_896A_98CF800AB084 */
