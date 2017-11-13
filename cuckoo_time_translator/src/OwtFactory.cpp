#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/KalmanOwt.h>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <cuckoo_time_translator/OwtFactory.h>

namespace cuckoo_time_translator {

OwtFactory::~OwtFactory() {
}

template <FilterAlgorithm::Type FA, typename Owt, typename Config>
OwtFactoryImpl<FA, Owt, Config>::OwtFactoryImpl(const Config & c) : Config(c) {
}

template <FilterAlgorithm::Type FA, typename Owt, typename Config>
FilterAlgorithm OwtFactoryImpl<FA, Owt, Config>::getFilterAlgorithm() const {
  return FA;
}

template <typename Owt, typename Config>
Owt * newOwt(const Config & c) {
  return new Owt(c);
}
template <typename Owt>
Owt * newOwt(const NoConfig &) {
  return new Owt();
}


template <FilterAlgorithm::Type FA, typename Owt, typename Config>
std::unique_ptr<OneWayTranslator> OwtFactoryImpl<FA, Owt, Config>::createOwt() const {
  return std::unique_ptr<OneWayTranslator>(newOwt<Owt>(static_cast<const Config &>(*this)));
}

template <FilterAlgorithm::Type FA, typename Owt, typename Config>
std::unique_ptr<OwtFactory> OwtFactoryImpl<FA, Owt, Config>::clone() const {
  return std::unique_ptr<OwtFactory>(new OwtFactoryImpl(*this));
}


// -------------------------------------------------

template class OwtFactoryImpl<FilterAlgorithm::Kalman, KalmanOwt, KalmanOwt::Config>;

Defaults & Defaults::setFilterConfig(const KalmanOwtConfig& c) {
  regOwtFactory(KalmanOwtFactory(c));
  return *this;
}

template class OwtFactoryImpl<FilterAlgorithm::ConvexHull, ConvexHullOwt>;
template class OwtFactoryImpl<FilterAlgorithm::ReceiveTimeOnly, NopOwt>;

}
