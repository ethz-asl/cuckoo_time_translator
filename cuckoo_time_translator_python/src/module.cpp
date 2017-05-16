#include <boost/python.hpp>
#include <boost/python/make_constructor.hpp>

#include <cuckoo_time_translator/OneWayTranslator.h>
#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/KalmanOwt.h>
#include <cuckoo_time_translator/SwitchingOwt.h>

using namespace boost::python;
using namespace cuckoo_time_translator;

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
    .def("isReady", &OneWayTranslator::isReady, "bool isReady() const")
    .def("translateToLocalTimestamp", &OneWayTranslator::translateToLocalTimestamp, "LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const")
    .def("updateAndTranslateToLocalTimestamp", &OneWayTranslator::updateAndTranslateToLocalTimestamp, "LocalTimestamp updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs)")
    ;

  class_<ConvexHullOwt, bases<OneWayTranslator>>("ConvexHullOwt", init<>())
    .def("getSkew", &ConvexHullOwt::getSkew, "double getSkew() const")
    .def("getOffset", &ConvexHullOwt::getOffset, "LocalTime getOffset() const")
    .def("getStackSize", &ConvexHullOwt::getStackSize, "size_t getStackSize() const")
    ;

  class_<KalmanOwt, bases<OneWayTranslator>>("KalmanOwt", init<>())
    ;

  class_<SwitchingOwt, bases<OneWayTranslator>, boost::noncopyable>("SwitchingOwt", init<double, const OneWayTranslator &>())
    .def("getSwitchingTimeSeconds", &SwitchingOwt::getSwitchingTimeSeconds, "double getSwitchingTimeSeconds() const")
    ;

}


BOOST_PYTHON_MODULE(libcuckoo_time_translator_python)
{
  exportTimestampOwts();
}
