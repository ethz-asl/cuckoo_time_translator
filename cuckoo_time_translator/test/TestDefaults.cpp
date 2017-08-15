#include <cuckoo_time_translator/DeviceTimeTranslator.h>
#include <cuckoo_time_translator/KalmanOwt.h>

#include <memory>

#include <gtest/gtest.h>

using namespace cuckoo_time_translator;

TEST(Defaults, KalmanDefaults) {
  Defaults d;
  auto uk = d.createOwt(FilterAlgorithm::Kalman);
  ASSERT_TRUE(uk.get());
  auto k = dynamic_cast<KalmanOwt*>(uk.get());
  ASSERT_TRUE(k);

  KalmanOwtConfig kc;
  EXPECT_EQ(kc.updateRate, k->getConfig().updateRate);

  kc.updateRate += 1.0;

  d.setFilterConfig(kc);

  uk = d.createOwt(FilterAlgorithm::Kalman);
  ASSERT_TRUE(uk.get());
  k = dynamic_cast<KalmanOwt*>(uk.get());
  ASSERT_TRUE(k);

  EXPECT_EQ(kc.updateRate, k->getConfig().updateRate);
}
