#ifndef HA1414207_EF1A_4368_BD21_A56BFFA7D0BB
#define HA1414207_EF1A_4368_BD21_A56BFFA7D0BB

#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <cuckoo_time_translator/KalmanOwt.h>
#include <memory>

namespace cuckoo_time_translator {

class OneWayTranslator;

class OwtFactory {
 public:
  virtual ~OwtFactory();
  virtual FilterAlgorithm getFilterAlgorithm() const = 0;
  virtual std::unique_ptr<OneWayTranslator> createOwt() const = 0;
  virtual std::unique_ptr<OwtFactory> clone() const = 0;
};

struct NoConfig {
};

template <FilterAlgorithm::Type FA, typename Owt, typename Config = NoConfig>
class OwtFactoryImpl : public OwtFactory, public Config {
 public:
  OwtFactoryImpl(const Config & c = Config());
  FilterAlgorithm getFilterAlgorithm() const override;
  std::unique_ptr<OneWayTranslator> createOwt() const override;
  std::unique_ptr<OwtFactory> clone() const override;
};

typedef OwtFactoryImpl<FilterAlgorithm::Kalman, KalmanOwt, KalmanOwt::Config> KalmanOwtFactory;
typedef OwtFactoryImpl<FilterAlgorithm::ConvexHull, ConvexHullOwt> ConvexHullOwtFactory;
typedef OwtFactoryImpl<FilterAlgorithm::ReceiveTimePassThrough, ReceiveTimePassThroughOwt> ReceiveTimePassThroughOwtFactory;
typedef OwtFactoryImpl<FilterAlgorithm::DeviceTimePassThrough, DeviceTimePassThroughOwt> DeviceTimePassThroughOwtFactory;
}

#endif /* HA1414207_EF1A_4368_BD21_A56BFFA7D0BB */
