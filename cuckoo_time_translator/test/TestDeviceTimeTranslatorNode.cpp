#include <cuckoo_time_translator/ConvexHullOwt.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <gtest/gtest.h>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "ros/ros.h"
#include <cuckoo_time_translator/DeviceTimestamp.h>
#pragma GCC diagnostic pop

#include "../src/MockCuckooDeviceDriver.h"

using namespace cuckoo_time_translator;

int count = 0;
struct Msg {
  FilterAlgorithm filterAlgo;
  TimePair timePair;
};
std::vector<Msg> msgs;

void clear(){
  msgs.clear();
}

void callback(const DeviceTimestamp::ConstPtr& msg)
{
  msgs.emplace_back(
      Msg{
        static_cast<FilterAlgorithm::Type>(msg->filter_algorithm),
        TimePair{RemoteTime(double(msg->event_stamp)), LocalTime(msg->receive_time.toSec())}
      }
  );
}

void receive(ros::Rate & loop_rate) {
  for (unsigned i = 0u; i < 10u; i++){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void changeAlgorithm(FilterAlgorithm fa){
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter filterAlgoParam;
  dynamic_reconfigure::Config & conf = srv_req.config;

  filterAlgoParam.name = "filter_algo";
  filterAlgoParam.value = static_cast<int>(fa);
  conf.ints.push_back(filterAlgoParam);

  ros::service::call("/device_time/set_parameters", srv_req, srv_resp);
}


TEST(DeviceTimeTranslator, ReceiveStamps) {
  constexpr int ExpectedMinimalMsgNum = 50;

  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe("/device_time", 10, callback);

  ros::Time expectedOffset = ros::Time::now() + ros::Duration(MockCuckooDeviceDriver::kOffset);

  receive(loop_rate);
  ASSERT_GT(msgs.size(), ExpectedMinimalMsgNum);

  ConvexHullOwt owt;
  for(auto & m : msgs){
    owt.updateAndTranslateToLocalTimestamp(RemoteTime(m.timePair.remote / MockCuckooDeviceDriver::kFreq), m.timePair.local);
  }
  EXPECT_NEAR(expectedOffset.toSec(), owt.getOffset(), 0.5);
  EXPECT_NEAR(MockCuckooDeviceDriver::kSkew, owt.getSkew(), 0.01);

  EXPECT_EQ(FilterAlgorithm::ReceiveTimePassThrough, msgs.back().filterAlgo.type);

  clear();
  ASSERT_EQ(msgs.size(), 0);

  changeAlgorithm(FilterAlgorithm::Kalman);
  receive(loop_rate);
  ASSERT_GT(msgs.size(), ExpectedMinimalMsgNum);

  EXPECT_EQ(FilterAlgorithm::Kalman, msgs.back().filterAlgo.type);
}
