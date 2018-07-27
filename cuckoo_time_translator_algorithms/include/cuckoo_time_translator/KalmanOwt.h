#ifndef HF8B63F63_F998_436E_92A5_21AB87EE5FC0
#define HF8B63F63_F998_436E_92A5_21AB87EE5FC0

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-register"
#include <Eigen/Dense>
#pragma clang diagnostic pop

#include "OneWayTranslator.h"

namespace cuckoo_time_translator {

struct KalmanOwtConfig {
  double sigmaInitOffset = 2e-3;
  double sigmaInitSkew = 1e-3;
  double sigmaOffset = 2e-3;
  double sigmaSkew = 2e-6;
  double updateCooldownSecs = 0.5;
  double outlierThreshold = 1.0;
};

class KalmanOwt : public OneWayTranslator
{
 public:
  typedef KalmanOwtConfig Config;

  KalmanOwt(Config config = Config());
  virtual ~KalmanOwt();
  virtual LocalTime translateToLocalTimestamp(RemoteTime device_time) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime device_time, LocalTime localTimeSecs) override;
  virtual bool isReadyToTranslate() const override;
  virtual void reset() override;

  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;

  const Config& getConfig() const { return config; }
  void setConfig(const Config& config);
 protected:
  virtual KalmanOwt* cloneImpl() const override;
 private:
  void initialize(double device_time, double localTimeSecs);
  void applyConfig();

  Config config;

  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  double R_;
  Eigen::Matrix<double, 1, 2> H_;
  bool isInitialized_;
  double lastUpdateDeviceTime_;
  double lastUpdateDt_;
};

}

#endif /* HF8B63F63_F998_436E_92A5_21AB87EE5FC0 */
