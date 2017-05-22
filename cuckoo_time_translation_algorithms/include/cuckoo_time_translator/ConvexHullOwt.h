#ifndef HD5867872_5846_4710_82B2_BDFD92BC89EA
#define HD5867872_5846_4710_82B2_BDFD92BC89EA

#include <vector>
#include <cstdint>

#include "AbstractAssert.h"
#include "OneWayTranslator.h"
#include "ConvexHull.h"

namespace cuckoo_time_translator {

class ConvexHullOwt : public OneWayTranslator {
 public:
  ConvexHullOwt();
  ConvexHullOwt(const ConvexHullOwt&);
  ConvexHullOwt(ConvexHullOwt&&);
  virtual ~ConvexHullOwt();
  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) override;
  virtual bool isReadyToTranslate() const override;
  virtual void reset() override;

  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;

  double getSkew() const;
  LocalTime getOffset() const;
  size_t getStackSize() const;
 private:
  ConvexHull<double> impl;
  virtual ConvexHullOwt* cloneImpl() const override;
};

}  // namespace cuckoo_time_translator

#endif /* HD5867872_5846_4710_82B2_BDFD92BC89EA */
