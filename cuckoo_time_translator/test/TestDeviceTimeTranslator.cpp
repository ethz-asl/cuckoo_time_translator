#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cuckoo_time_translator/DeviceTimestamp.h>
#include <ros/node_handle.h>
#pragma GCC diagnostic pop

using namespace cuckoo_time_translator;

TEST(DeviceTimeTranslator, DefaultDeviceTimeUnwrapperAndTranslator) {
  DefaultDeviceTimeUnwrapperAndTranslator translator({1000ul, 10.0}, "");
  translator.setFilterAlgorithm(FilterAlgorithm::ConvexHull);
  EXPECT_EQ(10.0, translator.update(100ul, ros::Time(10.0)).toSec());
  EXPECT_EQ(90.0, translator.update(900ul, ros::Time(90.0)).toSec());
  EXPECT_EQ(100.0, translator.update(0ul, ros::Time(110.0)).toSec());
}

TEST(DeviceTimeTranslator, DefaultDeviceTimeUnwrapperAndTranslatorWithTransmitTime) {
  DefaultDeviceTimeUnwrapperAndTranslatorWithTransmitTime translator({1000ul, 10.0}, "");
  translator.setFilterAlgorithm(FilterAlgorithm::ConvexHull);
  EXPECT_EQ(10.0, translator.update(90ul, 100ul, ros::Time(10.0)).toSec()); // passes through receive time because it is the first event
  EXPECT_EQ(89.0, translator.update(890ul, 900ul, ros::Time(90.0)).toSec());
  EXPECT_EQ(99.0, translator.update(990ul, 0ul, ros::Time(110.0)).toSec());
}

TEST(DeviceTimeTranslator, UnwrappedDeviceTimeTranslatorNoTranslation) {
  UnwrappedDeviceTimeTranslator translator({100.0}, "");
  translator.setFilterAlgorithm(FilterAlgorithm::None);
  EXPECT_EQ(10.0, translator.update(100ul, ros::Time(10.0)).toSec());
  EXPECT_EQ(110.0, translator.update(1000ul, ros::Time(110.0)).toSec());
  EXPECT_EQ(120.0, translator.update(0ul, ros::Time(120.0)).toSec());
}
