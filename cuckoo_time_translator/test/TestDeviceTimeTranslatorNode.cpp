#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "ros/ros.h"
#include <cuckoo_time_translator/DeviceTimestamp.h>
#pragma GCC diagnostic pop

#include "../src/MockCuckooDeviceDriver.h"

using namespace cuckoo_time_translator;

int count = 0;

std::vector<TimePair> timePairs;

void callback(const DeviceTimestamp::ConstPtr& msg)
{
  timePairs.emplace_back(TimePair{RemoteTime(double(msg->event_stamp)), LocalTime(msg->receive_time.toSec())});
}

TEST(DeviceTimeTranslator, ReceiveStamps) {
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/device_time", 10, callback);
  ros::Rate loop_rate(10);

  ros::Time expectedOffset = ros::Time::now() + ros::Duration(MockCuckooDeviceDriver::kOffset);

  for (unsigned i = 0u; i < 10u; i++){
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_GT(timePairs.size(), 50);

  ConvexHullOwt owt;
  for(auto & tp : timePairs){
    owt.updateAndTranslateToLocalTimestamp(RemoteTime(tp.remote / MockCuckooDeviceDriver::kFreq), tp.local);
  }
  EXPECT_NEAR(expectedOffset.toSec(), owt.getOffset(), 0.5);
  EXPECT_NEAR(MockCuckooDeviceDriver::kSkew, owt.getSkew(), 0.01);
}
