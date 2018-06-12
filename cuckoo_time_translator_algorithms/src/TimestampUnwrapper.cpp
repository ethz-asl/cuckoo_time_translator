#include <cuckoo_time_translator/TimestampUnwrapper.h>

#include <console_bridge/console.h>

namespace cuckoo_time_translator {

using namespace internal;

TimestampUnwrapper::~TimestampUnwrapper() {
}

ClockParameters::ClockParameters(double clockFrequencyHz) :
    clockFrequencyHz_(clockFrequencyHz)
{
}

WrappingClockParameters::WrappingClockParameters(uint64_t wrapAroundNumber, double clockFrequencyHz) :
    ClockParameters(clockFrequencyHz),
    maxStamp_(0u),
    wrapAroundNumber_(wrapAroundNumber)
{
}

WrappingClockParameters::~WrappingClockParameters() {
}

void WrappingClockParameters::checkNewDeviceStamp(uint64_t wrapsCounter, uint64_t newDeviceStamp) {
  maxStamp_ = std::max(newDeviceStamp, maxStamp_);
  if (newDeviceStamp >= wrapAroundNumber_){
    const uint64_t newWrapAroundNumber = newDeviceStamp + 1;
    CONSOLE_BRIDGE_logError("newDeviceStamp=%u is larger than wrapAroundNumber=%lu -> adapting wrapAroundNumber to %lu!", newDeviceStamp, wrapAroundNumber_, newWrapAroundNumber);
    wrapAroundNumber_ = newWrapAroundNumber;
  }
  if(wrapsCounter % 10 == 9 && uint64_t(maxStamp_) < wrapAroundNumber_ * 2u / 3u){
    CONSOLE_BRIDGE_logWarn("Last maxStamp=%u, suspiciously small! Maybe it wraps in fact earlier? (wrapAroundNumber=%lu)",  maxStamp_, wrapAroundNumber_);
  }
}


internal::Unwrapper::Unwrapper() :
    wrapsCounter_(0u),
    lastStamp_(0u)
{
}

void internal::Unwrapper::updateWithNewStamp(WrappingClockParameters & wrappingClockParameters, uint64_t newDeviceStamp) {
  wrappingClockParameters.checkNewDeviceStamp(wrapsCounter_, newDeviceStamp);
  if (lastStamp_ > newDeviceStamp) {
    wrapsCounter_ ++;
  }
  lastStamp_ = newDeviceStamp;
}

internal::AbstractTimestampUnwrapper::AbstractTimestampUnwrapper(const WrappingClockParameters & wrappingClockParameters) :
    clockParams_(wrappingClockParameters)
{
}

double internal::AbstractTimestampUnwrapper::stampToSec(UnwrappedStamp stamp) const {
  return clockParams_.stampToSec(stamp);
}

TimestampUnwrapperEventOnly::TimestampUnwrapperEventOnly(const WrappingClockParameters & wrappingClockParameters) :
    AbstractTimestampUnwrapper(wrappingClockParameters)
{
}

UnwrappedStamp TimestampUnwrapperEventOnly::getUnwrappedEventStamp() const {
  return eventUnwrapper_.getUnwrappedStamp(clockParams_);
}

UnwrappedStamp TimestampUnwrapperEventOnly::getUnwrappedTransmitStamp() const {
  return getUnwrappedEventStamp();
}

void TimestampUnwrapperEventOnly::updateWithNewEventStamp(uint64_t newDeviceStamp) {
  eventUnwrapper_.updateWithNewStamp(clockParams_, newDeviceStamp);
}

TimestampUnwrapperEventAndTransmit::TimestampUnwrapperEventAndTransmit(const WrappingClockParameters & wrappingClockParameters) :
    AbstractTimestampUnwrapper(wrappingClockParameters)
{
}

void TimestampUnwrapperEventAndTransmit::updateWithNewEventStamp(uint64_t newEventDeviceStamp) {
  eventUnwrapper_.updateWithNewStamp(clockParams_, newEventDeviceStamp);
}

void TimestampUnwrapperEventAndTransmit::updateWithNewTransmitStamp(uint64_t newTransmitDeviceStamp) {
  transmitUnwrapper_.updateWithNewStamp(clockParams_, newTransmitDeviceStamp);
}

UnwrappedStamp TimestampUnwrapperEventAndTransmit::getUnwrappedEventStamp() const {
  return eventUnwrapper_.getUnwrappedStamp(clockParams_);
}
UnwrappedStamp TimestampUnwrapperEventAndTransmit::getUnwrappedTransmitStamp() const {
  return transmitUnwrapper_.getUnwrappedStamp(clockParams_);
}

bool TimestampUnwrapperEventOnly::hasSeparateTransmitTime() const {
  return false;
}
bool TimestampUnwrapperEventAndTransmit::hasSeparateTransmitTime() const {
  return true;
}

TimestampPassThrough::TimestampPassThrough(const ClockParameters & clockParameters) :
    clockParams_(clockParameters),
    lastDeviceStamp_(0L)
{
}

TimestampPassThrough::~TimestampPassThrough() {
}

UnwrappedStamp TimestampPassThrough::getUnwrappedEventStamp() const {
  return lastDeviceStamp_;
}

UnwrappedStamp TimestampPassThrough::getUnwrappedTransmitStamp() const {
  return lastDeviceStamp_;
}

void TimestampPassThrough::updateWithNewEventStamp(Timestamp newDeviceStamp) {
  lastDeviceStamp_ = newDeviceStamp;
}

double TimestampPassThrough::stampToSec(UnwrappedStamp stamp) const {
  return clockParams_.stampToSec(stamp);
}

double TimestampPassThrough::getClockFrequencyHz() const {
  return clockParams_.getClockFrequencyHz();
}

bool TimestampPassThrough::hasSeparateTransmitTime() const {
  return false;
}

}  // namespace cuckoo_time_translator
