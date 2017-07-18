#include <cuckoo_time_translator/OneWayTranslator.h>

#include <iostream>
#include <stdexcept>

#include <cuckoo_time_translator/AbstractAssert.h>

namespace cuckoo_time_translator {

double TimePair::update(OneWayTranslator & owt) const {
  return owt.updateAndTranslateToLocalTimestamp(remote, local);
}

OneWayTranslator::~OneWayTranslator() {
}

std::unique_ptr<OneWayTranslator> OneWayTranslator::clone() const {
  return std::unique_ptr<OneWayTranslator>(cloneImpl());
}

NopOwt::~NopOwt() {
}

LocalTime NopOwt::translateToLocalTimestamp(RemoteTime /*remoteTimeTics*/) const {
  AASSERT(false, "translateToLocalTimestamp is not implemented in NopOwt!");
  throw ""; // dummy;
}

LocalTime NopOwt::updateAndTranslateToLocalTimestamp(RemoteTime /*remoteTimeTics*/, LocalTime localTimeSecs) {
  return localTimeSecs;
}

bool NopOwt::isReadyToTranslate() const {
  return false;
}

void NopOwt::printNameAndConfig(std::ostream& o) const {
  o << "NopOwt()";
}

void NopOwt::printState(std::ostream& /*o*/) const {
}

void NopOwt::reset() {
}

NopOwt* NopOwt::cloneImpl() const {
  return new NopOwt();
}

} /* namespace cuckoo_time_translator */
