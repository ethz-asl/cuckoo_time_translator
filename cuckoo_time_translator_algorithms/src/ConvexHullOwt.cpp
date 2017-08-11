#include <cuckoo_time_translator/ConvexHullOwt.h>

#include <iostream>

#include <cuckoo_time_translator/ConvexHullImpl.h>

namespace cuckoo_time_translator {

template class ConvexHull<double>;
template class ConvexHull<std::int64_t>;
template class ConvexHull<std::uint64_t>;

// defaults necessary in cpp because otherwise ConvexHull is going to be incomplete for the client packages
ConvexHullOwt::ConvexHullOwt() = default;
ConvexHullOwt::ConvexHullOwt(const ConvexHullOwt&) = default;
ConvexHullOwt::ConvexHullOwt(ConvexHullOwt&&) = default;
ConvexHullOwt::~ConvexHullOwt() = default;

LocalTime ConvexHullOwt::translateToLocalTimestamp(RemoteTime remoteTimeTics) const {
  return LocalTime(impl.getLocalTime(remoteTimeTics));
}

LocalTime ConvexHullOwt::updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) {
  return LocalTime(impl.correctTimestamp(remoteTimeTics, localTimeSecs));
}

bool ConvexHullOwt::isReadyToTranslate() const {
  return impl.convexHullSize() >= 2u;
}

void ConvexHullOwt::printNameAndConfig(std::ostream& o) const {
  o << "ConvexHull()";
}
void ConvexHullOwt::printState(std::ostream& o) const {
  o << "offset=" << std::fixed << getOffset() << ", skew=" << getSkew() << ", stackSize=" << getStackSize();
}

void ConvexHullOwt::reset() {
  impl.reset();
}

double cuckoo_time_translator::ConvexHullOwt::getSkew() const {
  return impl.getSlope();
}

LocalTime cuckoo_time_translator::ConvexHullOwt::getOffset() const {
  return LocalTime(impl.getOffset());
}

size_t cuckoo_time_translator::ConvexHullOwt::getStackSize() const {
  return impl.convexHullSize();
}

ConvexHullOwt* ConvexHullOwt::cloneImpl() const {
  return new ConvexHullOwt(*this);
}

}
