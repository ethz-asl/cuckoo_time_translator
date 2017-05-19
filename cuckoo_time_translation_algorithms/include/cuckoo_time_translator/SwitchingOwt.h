#ifndef H63A7D7A2_B361_4B2F_ACA8_8E773C328ABE
#define H63A7D7A2_B361_4B2F_ACA8_8E773C328ABE

#include <array>
#include <memory>
#include <functional>

#include "OneWayTranslator.h"

namespace cuckoo_time_translator {

class SwitchingOwt : public OneWayTranslator {
 public:
  SwitchingOwt(double switchingTimeSeconds, const OneWayTranslator & blueprint);
  SwitchingOwt(double switchingTimeSeconds, std::function<std::unique_ptr<OneWayTranslator>()> owtFactory);

  SwitchingOwt(const SwitchingOwt & other);
  SwitchingOwt(SwitchingOwt &&) = default;

  template <typename Owt>
  static SwitchingOwt craeteSwitchingOwt(const double switchingTimeSeconds){
    return SwitchingOwt(switchingTimeSeconds, []() { return std::move(std::unique_ptr<OneWayTranslator>(new Owt)); });
  }

  virtual ~SwitchingOwt();

  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) override;
  virtual bool isReadyToTranslate() const override;
  virtual void reset() override;

  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;

  double getSwitchingTimeSeconds() const {
    return switchingTimeSeconds_;
  }

  void setSwitchingTimeSeconds(double switchingTimeSeconds) {
    this->switchingTimeSeconds_ = switchingTimeSeconds;
  }

  uint32_t getSwitchCount() const {
    return switchCount_;
  }

  LocalTime getLastSwitchTime() const {
    return LocalTime(lastSwitchTime_);
  }

  const OneWayTranslator & getCurrentOwt() const;
  const OneWayTranslator & getPendingOwt() const;
  OneWayTranslator & getCurrentOwt();
  OneWayTranslator & getPendingOwt();

 protected:
  virtual SwitchingOwt* cloneImpl() const override;
 private:
  double switchingTimeSeconds_, lastSwitchTime_ = -1;
  uint32_t switchCount_ = 0;
  std::array<std::unique_ptr<OneWayTranslator>, 2> oneWayTranslators_;

  void switchOwts();
};

} /* namespace cuckoo_time_translator */

#endif /* H63A7D7A2_B361_4B2F_ACA8_8E773C328ABE */
