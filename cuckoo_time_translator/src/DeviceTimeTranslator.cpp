#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#include <cinttypes>

#include <boost/bind/bind.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/this_node.h>
#include <dynamic_reconfigure/server.h>
#pragma GCC diagnostic pop

#include <cuckoo_time_translator/OneWayTranslator.h>
#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/KalmanOwt.h>
#include <cuckoo_time_translator/SwitchingOwt.h>
#include <cuckoo_time_translator/OwtFactory.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cuckoo_time_translator/DeviceTimeTranslatorConfig.h>
#include <cuckoo_time_translator/DeviceTimestamp.h>
#pragma GCC diagnostic pop

namespace cuckoo_time_translator {

const char * const NS::kDefaultSubnamespace = "device_time";

NS::NS(const std::string& nameSpace, bool appendDeviceTimeSubnamespace)
  : NS(nameSpace, (appendDeviceTimeSubnamespace ? std::string(kDefaultSubnamespace) : ""))
{
}

std::string addHeadSlashIfNecessary(const std::string & name){
  if (!name.empty() && name[0] != '/'){
    return "/" + name;
  } else {
    return name;
  }
}

NS::NS(const std::string& nameSpace, const std::string& subNameSpace) :
  nameSpace_(nameSpace + addHeadSlashIfNecessary(subNameSpace))
{
}

NS::NS(const char* nameSpace, const char* subNameSpace)
    : NS(std::string(nameSpace), std::string(subNameSpace)) {
}

NS::NS(const char* nameSpace, bool appendDeviceTimeSubnamespace)
    : NS(std::string(nameSpace), appendDeviceTimeSubnamespace) {
}


class Defaults::Impl {
 public:
  Impl(){
    regOwtFactory(new ReceiveTimePassThroughOwtFactory);
    regOwtFactory(new KalmanOwtFactory);
    regOwtFactory(new ConvexHullOwtFactory);
    regOwtFactory(new DeviceTimePassThroughOwtFactory);
  }

  Impl(const Impl& other){
    for(auto & e : other.owtFactoryReg_){
      owtFactoryReg_.emplace(e.first, e.second->clone());
    }
  }

  void setFilterAlgorithm(FilterAlgorithm filterAlgorithm){
    setParam("filter_algo", static_cast<int>(filterAlgorithm));
  }
  void setSwitchTimeSecs(double secs){
    setParam("switch_time", secs);
  }
  void apply(ros::NodeHandle & nh) const{
    for(auto && op: operations){
      op.second(nh, op.first);
    }
  }

  void regOwtFactory(OwtFactory * owtFactoryPtr) {
    owtFactoryReg_[owtFactoryPtr->getFilterAlgorithm()].reset(owtFactoryPtr);
  }

  std::unique_ptr<OneWayTranslator> createOwt(FilterAlgorithm fa) const;
 private:
  template <typename T>
  void setParam(const std::string & name, T value){
    operations.emplace(name, [value](ros::NodeHandle & nh, const std::string & name){
      if(!nh.hasParam(name)){
        ROS_INFO("Setting device time translator parameter '%s' to %s (in %s).", name.c_str(), std::to_string(value).c_str(), nh.getNamespace().c_str());
        nh.setParam(name, value);
      } else {
        T currentValue;
        nh.getParam(name, currentValue);
        ROS_INFO("NOT setting device time translator parameter '%s' to %s, because it is already set to %s.", name.c_str(), std::to_string(value).c_str(), std::to_string(currentValue).c_str());
      }
    });
  }
  std::map<std::string, std::function<void(ros::NodeHandle & nh, const std::string & name)>> operations;
  std::map<FilterAlgorithm::Type, std::unique_ptr<OwtFactory>> owtFactoryReg_;
};

Defaults::Defaults() :
  pImpl_(new Impl()){
}

Defaults::Defaults(const Defaults & other) :
  pImpl_(new Impl(*other.pImpl_)) {
}

Defaults::~Defaults()
{
  delete pImpl_;
}

Defaults & Defaults::setFilterAlgorithm(FilterAlgorithm filterAlgorithm) {
  pImpl_->setFilterAlgorithm(filterAlgorithm);
  return *this;
}

Defaults & Defaults::setSwitchTimeSecs(double secs) {
  pImpl_->setSwitchTimeSecs(secs);
  return *this;
}

void Defaults::regOwtFactory_(OwtFactory* owtFactoryPtr) {
  getImpl().regOwtFactory(owtFactoryPtr);
}

Defaults::Impl& Defaults::getImpl() {
  return *pImpl_;
}

const Defaults::Impl& Defaults::getImpl() const {
  return *pImpl_;
}

std::unique_ptr<OneWayTranslator> Defaults::Impl::createOwt(FilterAlgorithm fa) const {
  auto e = owtFactoryReg_.find(fa);
  if(e != owtFactoryReg_.end()){
    return e->second->createOwt();
  } else {
    ROS_ERROR("Unknown device time filter algorithm : %u. Falling back to default translator (ReceiveTimePassThroughOwt).", static_cast<unsigned>(fa));
    return std::unique_ptr<OneWayTranslator>(new ReceiveTimePassThroughOwt);
  }
}


std::unique_ptr<OneWayTranslator> Defaults::createOwt(FilterAlgorithm::Type fa) const {
  return getImpl().createOwt(fa);
}

class DeviceTimeTranslator::Impl {
 public:
  Impl(const std::string & nameSpace, const Defaults & defaults) : defaults_(defaults), nh_(nameSpace), srv_((defaults.getImpl().apply(nh_), nh_))
  {
  }

  OneWayTranslator & getTimeTranslator(){
    updateTranslator();
    return *timeTranslator_;
  }

  FilterAlgorithm getCurrentAlgo() const {
    return currentAlgo_;
  }

  FilterAlgorithm getExpectedAlgo() const {
    return expectedAlgo_;
  }

  void setExpectedAlgo(FilterAlgorithm expectedAlgo) {
    expectedAlgo_ = expectedAlgo;
  }

  ros::Publisher& getDeviceTimePub() {
    return deviceTimePub_;
  }

  dynamic_reconfigure::Server<DeviceTimeTranslatorConfig>& getConfigSrv() {
    return srv_;
  }

  DeviceTimestamp& getMsg() {
    return msg;
  }

  double getExpectedSwitchingTimeSeconds() const {
    return expectedSwitchingTimeSeconds_;
  }

  void setExpectedSwitchingTimeSeconds(double expectedSwitchingTimeSeconds = 0) {
    expectedSwitchingTimeSeconds_ = expectedSwitchingTimeSeconds;
  }

  ros::NodeHandle& getNh() {
    return nh_;
  }

  const OneWayTranslator* getCurrentOwt() const {
    return timeTranslator_.get();
  }

  void resetTranslation() {
    timeTranslator_.reset();
  }
 private:

  template <typename Owt>
  OneWayTranslator* createOwt(){
    if (shouldSwitch(switchingTimeSeconds_)) {
      return new SwitchingOwt(SwitchingOwt::craeteSwitchingOwt<Owt>(switchingTimeSeconds_));
    } else {
      return new Owt;
    }
  }

  void updateTranslator() {
    const FilterAlgorithm expectedAlgo = expectedAlgo_;
    const double expectedSwitchingTimeSeconds = expectedSwitchingTimeSeconds_;
    bool somethingWasUpdated = false;

    if (timeTranslator_ == nullptr || currentAlgo_ != expectedAlgo || (shouldSwitch(expectedSwitchingTimeSeconds) != shouldSwitch(switchingTimeSeconds_))) {
      timeTranslator_.reset();
      switchingTimeSeconds_ = expectedSwitchingTimeSeconds;
      somethingWasUpdated = true;
      timeTranslator_ = defaults_.createOwt(expectedAlgo_);
      if(dynamic_cast<ReceiveTimePassThroughOwtFactory*>(timeTranslator_.get()) == nullptr
          && dynamic_cast<DeviceTimePassThroughOwtFactory*>(timeTranslator_.get()) == nullptr
          && shouldSwitch(expectedSwitchingTimeSeconds)){
        timeTranslator_ = std::unique_ptr<OneWayTranslator>(new SwitchingOwt(expectedSwitchingTimeSeconds, [this]() { return defaults_.createOwt(expectedAlgo_); }));
      }
      switchingTimeSeconds_ = expectedSwitchingTimeSeconds;
      currentAlgo_ = expectedAlgo;
    }
    else if (expectedSwitchingTimeSeconds != switchingTimeSeconds_) {
      auto switchingOwt = dynamic_cast<SwitchingOwt*>(timeTranslator_.get());
      if(switchingOwt){
        switchingOwt->setSwitchingTimeSeconds(expectedSwitchingTimeSeconds);
        somethingWasUpdated = true;
      }
      switchingTimeSeconds_ = expectedSwitchingTimeSeconds;
    }

    if(somethingWasUpdated){
      std::stringstream ss;
      timeTranslator_->printNameAndConfig(ss);
      ROS_INFO("Using device time filter : %s.", ss.str().c_str());
    }
  }

  bool shouldSwitch(const double expectedSwitchingTimeSeconds) {
    return expectedSwitchingTimeSeconds > 0;
  }

  std::unique_ptr<OneWayTranslator> timeTranslator_;
  FilterAlgorithm currentAlgo_ = FilterAlgorithm::ReceiveTimePassThrough;
  FilterAlgorithm expectedAlgo_ = FilterAlgorithm::ReceiveTimePassThrough;
  Defaults defaults_;
  ros::Publisher deviceTimePub_;
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<DeviceTimeTranslatorConfig> srv_;

  double switchingTimeSeconds_ = 0, expectedSwitchingTimeSeconds_ = 0;
  DeviceTimestamp msg;
};

double DeviceTimeTranslator::getExpectedSwitchingTimeSeconds() const {
  return pImpl_->getExpectedSwitchingTimeSeconds();
}
void DeviceTimeTranslator::setExpectedSwitchingTimeSeconds(double expectedSwitchingTimeSeconds) {
  pImpl_->setExpectedSwitchingTimeSeconds(expectedSwitchingTimeSeconds);
}

void DeviceTimeTranslator::configCallback(DeviceTimeTranslatorConfig &config, uint32_t /*level*/)
{
  pImpl_->setExpectedAlgo(FilterAlgorithm::Type(config.filter_algo));
  pImpl_->setExpectedSwitchingTimeSeconds(config.switch_time);
}

DeviceTimeTranslator::DeviceTimeTranslator(const NS & nameSpace, const Defaults & defaults) :
    pImpl_(new Impl(nameSpace, defaults))
{
  ROS_INFO("DeviceTimeTranslator is going to publishing device timestamps on %s.", pImpl_->getNh().getNamespace().c_str());
  pImpl_->getDeviceTimePub() = pImpl_->getNh().advertise<DeviceTimestamp>("", 5);
  pImpl_->getConfigSrv().setCallback(boost::bind(&DeviceTimeTranslator::configCallback, this, _1, _2));

  if(pImpl_->getExpectedAlgo() == FilterAlgorithm::ReceiveTimePassThrough){
    ROS_WARN("Current %s/filterAlgo setting (=%u ~ ReceiveTimePassThrough) causes the sensor's hardware clock to be ignore. Instead the receive time in the driver is used as translated timestamp.", nameSpace.toString().c_str(), unsigned(FilterAlgorithm::ReceiveTimePassThrough));
  }
  if(pImpl_->getExpectedAlgo() == FilterAlgorithm::DeviceTimePassThrough){
    ROS_WARN("Current %s/filterAlgo setting (=%u ~ DeviceTimePassThrough) causes the receive time to be ignore. Instead the device time is passed through as translated timestamp.", nameSpace.toString().c_str(), unsigned(FilterAlgorithm::DeviceTimePassThrough));
  }
}

DeviceTimeTranslator::~DeviceTimeTranslator() {
  delete pImpl_;
}

FilterAlgorithm DeviceTimeTranslator::getCurrentFilterAlgorithm() const {
  return pImpl_->getCurrentAlgo();
}

const OneWayTranslator* DeviceTimeTranslator::getCurrentOwt() const {
  return pImpl_->getCurrentOwt();
}

void DeviceTimeTranslator::setFilterAlgorithm(FilterAlgorithm filterAlgorithm) {
  pImpl_->setExpectedAlgo(filterAlgorithm);
}

void DeviceTimeTranslator::resetTranslation() {
  pImpl_->resetTranslation();
}

ros::Time DeviceTimeTranslator::update(const TimestampUnwrapper & timestampUnwrapper, const ros::Time & receiveTime, const double offsetSecs) {
  if(!pImpl_) return receiveTime;

  auto & timeTranslator = pImpl_->getTimeTranslator();

  double translatedTime = timeTranslator.updateAndTranslateToLocalTimestamp(RemoteTime(timestampUnwrapper.getTransmitStampSec()), LocalTime(receiveTime.toSec()));

  auto & msg = pImpl_->getMsg();
  if(timestampUnwrapper.hasSeparateTransmitTime()){
    msg.transmit_stamp = timestampUnwrapper.getUnwrappedTransmitStamp().getValue();
    if(timeTranslator.isReadyToTranslate()){
      translatedTime = timeTranslator.translateToLocalTimestamp(RemoteTime(timestampUnwrapper.getEventStampSec()));
    }
  }
  float correction_secs;
  if (timeTranslator.isReadyToTranslate()){
    msg.header.stamp.fromSec(translatedTime);
    correction_secs = float(receiveTime.toSec() - translatedTime);
  } else {
    msg.header.stamp = receiveTime;
    correction_secs = 0.f;
  }

  msg.header.stamp += ros::Duration(offsetSecs);

  if(pImpl_->getDeviceTimePub().getNumSubscribers()){
    msg.event_stamp = timestampUnwrapper.getUnwrappedEventStamp().getValue();
    msg.receive_time = receiveTime;
    msg.offset_secs = offsetSecs;
    msg.correction_secs = correction_secs;
    msg.filter_algorithm = uint8_t(pImpl_->getCurrentAlgo().type);
    pImpl_->getDeviceTimePub().publish(msg);
  }
  ROS_DEBUG("Device time %" PRIu64 " + receive time %10.6f sec mapped to %10.6f sec (receive - translated = %.3f ms).", timestampUnwrapper.getUnwrappedEventStamp().getValue(), receiveTime.toSec(), translatedTime, (receiveTime.toSec() - translatedTime) * 1000);
  return msg.header.stamp;
}

ros::Time DeviceTimeTranslator::translate(const TimestampUnwrapper & timestampUnwrapper, UnwrappedStamp unwrappedEventStamp) const {
  return ros::Time(pImpl_->getTimeTranslator().translateToLocalTimestamp(RemoteTime(timestampUnwrapper.stampToSec(unwrappedEventStamp))));
}

bool DeviceTimeTranslator::isReadyToTranslate() const{
  return pImpl_->getTimeTranslator().isReadyToTranslate();
}

template <typename Unwrapper>
DeviceTimeUnwrapperAndTranslator<Unwrapper>::DeviceTimeUnwrapperAndTranslator(const UnwrapperClockParameters & clockParameters, const NS & nameSpace, const Defaults & defaults) :
    timestampUnwrapper(clockParameters),
    translator(nameSpace, defaults)
{
}

template <typename Unwrapper>
ros::Time DeviceTimeUnwrapperAndTranslator<Unwrapper>::update(Timestamp eventStamp, const ros::Time & receiveTime, double offset) {
  timestampUnwrapper.updateWithNewEventStamp(eventStamp);
  return translator.update(timestampUnwrapper, receiveTime, offset);
}

template <typename Unwrapper>
ros::Time DeviceTimeUnwrapperAndTranslatorWithTransmitTime<Unwrapper>::update(Timestamp eventStamp, Timestamp transmitStamp, const ros::Time & receiveTime, double offset) {
  this->timestampUnwrapper.updateWithNewEventStamp(eventStamp);
  this->timestampUnwrapper.updateWithNewTransmitStamp(transmitStamp);
  return this->translator.update(this->timestampUnwrapper, receiveTime, offset);
}

template <typename Unwrapper>
ros::Time DeviceTimeUnwrapperAndTranslator<Unwrapper>::translate(UnwrappedStamp unwrappedStamp) const {
  return translator.translate(timestampUnwrapper, unwrappedStamp);
}

template <typename Unwrapper>
bool DeviceTimeUnwrapperAndTranslator<Unwrapper>::isReadyToTranslate() const {
  return translator.isReadyToTranslate();
}

template <typename Unwrapper>
UnwrappedStamp DeviceTimeUnwrapperAndTranslator<Unwrapper>::unwrapEventStamp(typename Unwrapper::Timestamp eventStamp) {
  timestampUnwrapper.updateWithNewEventStamp(eventStamp);
  return timestampUnwrapper.getUnwrappedEventStamp();
}

template<typename Unwrapper_>
DeviceTimeUnwrapperAndTranslatorWithTransmitTime<Unwrapper_>::DeviceTimeUnwrapperAndTranslatorWithTransmitTime(const UnwrapperClockParameters& clockParameters, const NS& nameSpace, const Defaults & defaults) :
  DeviceTimeUnwrapperAndTranslator<Unwrapper_>(clockParameters, nameSpace, defaults)
{
}

template <typename Unwrapper>
UnwrappedStamp DeviceTimeUnwrapperAndTranslatorWithTransmitTime<Unwrapper>::unwrapTransmitStamp(Timestamp eventStamp) {
  this->timestampUnwrapper.updateWithNewTransmitStamp(eventStamp);
  return this->timestampUnwrapper.getUnwrappedTransmitStamp();
}

template class DeviceTimeUnwrapperAndTranslator<TimestampUnwrapperEventOnly>;
template class DeviceTimeUnwrapperAndTranslator<TimestampPassThrough>;
template class DeviceTimeUnwrapperAndTranslatorWithTransmitTime<TimestampUnwrapperEventAndTransmit>;

}
