#include <cuckoo_time_translator/TimestampUnwrapper.h>

#include <console_bridge/console.h>

namespace cuckoo_time_translator {

TimestampUnwrapper::TimestampUnwrapper(uint64_t wrapAroundNumber, double clockFrequencyHz) :
    wrapsCounter_(0u),
    lastStamp_(0u),
    maxStamp_(0u),
    wrapAroundNumber_(wrapAroundNumber),
    clockFrequencyHz_(clockFrequencyHz)
{
}

void TimestampUnwrapper::updateWithNewStamp(uint32_t newStamp) {
  maxStamp_ = std::max(newStamp, maxStamp_);
  if (newStamp >= wrapAroundNumber_){
    const uint64_t newWrapAroundNumber = uint64_t(newStamp) + 1;
    logError("newStamp=%u is larger than wrapAroundNumber=%lu -> adapting wrapAroundNumber to %lu!", newStamp, wrapAroundNumber_, newWrapAroundNumber);
    wrapAroundNumber_ = newWrapAroundNumber;
  }
  if (lastStamp_ > newStamp) {
    if(wrapsCounter_ % 10 == 9 && uint64_t(maxStamp_) < wrapAroundNumber_ * 2u / 3u){
      logWarn("Last maxStamp=%u, suspiciously small! Maybe it wraps in fact earlier? (wrapAroundNumber=%lu)",  maxStamp_, wrapAroundNumber_);
    }
    wrapsCounter_ ++;
  }
  lastStamp_ = newStamp;
}

} // namespace cuckoo_time_translator
