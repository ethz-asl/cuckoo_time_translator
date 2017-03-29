#include <cuckoo_time_translator/OneWayTranslator.h>

#include <iostream>
#include <stdexcept>

namespace cuckoo_time_translator {

double TimePair::update(OneWayTranslator & owt) const {
  return owt.updateAndTranslateToLocalTimestamp(remote, local);
}

OneWayTranslator::~OneWayTranslator() {
}

NopOwt::~NopOwt() {
}

LocalTime NopOwt::translateToLocalTimestamp(RemoteTime /*remoteTimeTics*/) const {
  throw std::runtime_error("translateToLocalTimestamp is not implemented in NopOwt!");
}

LocalTime NopOwt::updateAndTranslateToLocalTimestamp(RemoteTime /*remoteTimeTics*/, LocalTime localTimeSecs) {
  return localTimeSecs;
}

bool NopOwt::isReady() const {
  return false;
}

void NopOwt::printNameAndConfig(std::ostream& o) const {
  o << "NopOwt()";
}

void NopOwt::printState(std::ostream& /*o*/) const {
}

void NopOwt::reset() {
}
} /* namespace cuckoo_time_translator */
