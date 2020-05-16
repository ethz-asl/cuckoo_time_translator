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

ReceiveTimePassThroughOwt::~ReceiveTimePassThroughOwt() {
}

LocalTime ReceiveTimePassThroughOwt::translateToLocalTimestamp(RemoteTime /*remoteTimeTics*/) const {
  AASSERT(false, "translateToLocalTimestamp is not implemented in ReceiveTimePassThroughOwt!");
  throw ""; // dummy;
}

LocalTime ReceiveTimePassThroughOwt::updateAndTranslateToLocalTimestamp(RemoteTime /*remoteTimeTics*/, LocalTime localTimeSecs) {
  return localTimeSecs;
}

bool ReceiveTimePassThroughOwt::isReadyToTranslate() const {
  return true;
}

void ReceiveTimePassThroughOwt::printNameAndConfig(std::ostream& o) const {
  o << "ReceiveTimePassThroughOwt()";
}

void ReceiveTimePassThroughOwt::printState(std::ostream& /*o*/) const {
}

void ReceiveTimePassThroughOwt::reset() {
}

ReceiveTimePassThroughOwt* ReceiveTimePassThroughOwt::cloneImpl() const {
  return new ReceiveTimePassThroughOwt();
}

DeviceTimePassThroughOwt::~DeviceTimePassThroughOwt() {
}

LocalTime DeviceTimePassThroughOwt::translateToLocalTimestamp(RemoteTime remoteTimeTics) const {
  return LocalTime(static_cast<double>(remoteTimeTics));
}

LocalTime DeviceTimePassThroughOwt::updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime /*localTimeSecs*/) {
  return translateToLocalTimestamp(remoteTimeTics);
}

bool DeviceTimePassThroughOwt::isReadyToTranslate() const {
  return true;
}

void DeviceTimePassThroughOwt::printNameAndConfig(std::ostream& o) const {
  o << "DeviceTimePassThroughOwt()";
}

void DeviceTimePassThroughOwt::printState(std::ostream& /*o*/) const {
}

void DeviceTimePassThroughOwt::reset() {
}

DeviceTimePassThroughOwt* DeviceTimePassThroughOwt::cloneImpl() const {
  return new DeviceTimePassThroughOwt();
}


} /* namespace cuckoo_time_translator */
