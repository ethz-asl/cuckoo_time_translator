#include <cuckoo_time_translator/ConvexHullOwt.h>

namespace cuckoo_time_translator {

template class ConvexHullOwtT<double>;

ConvexHullOwt::~ConvexHullOwt() {
}

LocalTime ConvexHullOwt::translateToLocalTimestamp(RemoteTime remoteTimeTics) const {
  return LocalTime(impl.getLocalTime(remoteTimeTics));
}

LocalTime ConvexHullOwt::updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) {
  return LocalTime(impl.correctTimestamp(remoteTimeTics, localTimeSecs));
}

bool ConvexHullOwt::isReady() const {
  return impl.convexHullSize() >= 2u;
}

void ConvexHullOwt::printNameAndConfig(std::ostream& o) const {
  o << "ConvexHull()";
}
void ConvexHullOwt::printState(std::ostream& o) const {
  impl.printHullPoints(o);
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

}

