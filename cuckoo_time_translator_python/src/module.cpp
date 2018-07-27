#include <boost/python.hpp>
#include <boost/python/make_constructor.hpp>

#include <cuckoo_time_translator/OneWayTranslator.h>
#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/KalmanOwt.h>
#include <cuckoo_time_translator/SwitchingOwt.h>

using namespace boost::python;
using namespace cuckoo_time_translator;

template < void (OneWayTranslator::* Method) (std::ostream & o) const>
std::string getString(const OneWayTranslator * owt){
  std::stringstream ss;
  (owt->*Method)(ss);
  return ss.str();
}

void exportTimestampOwts()
{
  class_<TaggedTime>("LocalTime", init<double>())
    .def("__eq__", &TaggedTime::operator ==, "")
    .def("__ne__", &TaggedTime::operator !=, "")
    .def("__float__", &TaggedTime::operator double, "local time in seconds")
    ;

  class_<LocalTime, bases<TaggedTime>>("LocalTime", init<double>())
    ;
  class_<RemoteTime, bases<TaggedTime>>("RemoteTime", init<double>())
    ;

  class_<OneWayTranslator, boost::noncopyable>("OneWayTranslator", no_init)
    .def("reset", &OneWayTranslator::reset, "void reset()")
    .def("isReady", &OneWayTranslator::isReadyToTranslate, "bool isReady() const")
    .def("translateToLocalTimestamp", &OneWayTranslator::translateToLocalTimestamp, "LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const")
    .def("updateAndTranslateToLocalTimestamp", &OneWayTranslator::updateAndTranslateToLocalTimestamp, "LocalTimestamp updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs)")
    .def("getNameAndConfigString", &getString<&OneWayTranslator::printNameAndConfig>, "std::string getNameAndConfig() const")
    .def("getStateString", &getString<&OneWayTranslator::printState>, "std::string getStateString() const")
    ;

  class_<ConvexHullOwt, bases<OneWayTranslator>>("ConvexHullOwt", init<>())
    .def("getSkew", &ConvexHullOwt::getSkew, "double getSkew() const")
    .def("getOffset", &ConvexHullOwt::getOffset, "LocalTime getOffset() const")
    .def("getStackSize", &ConvexHullOwt::getStackSize, "size_t getStackSize() const")
    ;

  class_<KalmanOwt::Config>("KalmanOwtConfig", init<>())
    .def_readwrite("sigmaInitOffset", &KalmanOwt::Config::sigmaInitOffset)
    .def_readwrite("sigmaInitSkew", &KalmanOwt::Config::sigmaInitSkew)
    .def_readwrite("sigmaOffset", &KalmanOwt::Config::sigmaOffset)
    .def_readwrite("sigmaSkew", &KalmanOwt::Config::sigmaSkew)
    .def_readwrite("updateCooldownSecs", &KalmanOwt::Config::updateCooldownSecs)
    .def_readwrite("outlierThreshold", &KalmanOwt::Config::outlierThreshold)
  ;

  class_<KalmanOwt, bases<OneWayTranslator>>("KalmanOwt", init<>())
    .def("getConfig", &KalmanOwt::getConfig, "const Config& getConfig() const", return_value_policy<boost::python:: copy_const_reference>())
    .def("setConfig", &KalmanOwt::setConfig, "void setConfig(const Config& config)")
    ;

  class_<SwitchingOwt, bases<OneWayTranslator>, boost::noncopyable>("SwitchingOwt", init<double, const OneWayTranslator &>())
    .def("getSwitchingTimeSeconds", &SwitchingOwt::getSwitchingTimeSeconds, "double getSwitchingTimeSeconds() const")
    .def("getCurrentOwt", static_cast<OneWayTranslator& (SwitchingOwt::*)()>(&SwitchingOwt::getCurrentOwt), return_internal_reference<>(), "OneWayTranslator& getCurrentOwt()")
    ;

}


BOOST_PYTHON_MODULE(libcuckoo_time_translator_python)
{
  exportTimestampOwts();
}
