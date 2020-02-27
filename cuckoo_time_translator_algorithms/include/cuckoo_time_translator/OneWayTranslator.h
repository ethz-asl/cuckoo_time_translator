#ifndef H3A613172_9ECF_45BC_8D47_C711DE333A50
#define H3A613172_9ECF_45BC_8D47_C711DE333A50

#include <iosfwd>
#include <memory>

#include "Time.h"

namespace cuckoo_time_translator {

class OneWayTranslator {
 public:
  virtual ~OneWayTranslator();

  /**
   * Translate remoteTimeTics into localTimeSecs
   * This is allowed to throw an exception iff not @see isReady()
   * @param remoteTimeTics
   * @return localTimeSecs corresponding to remoteTimeTics
   */
  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const = 0;

  /**
   * Learn for a new remoteTimeTics - localTimeSecs pair and return localTimeSecs corresponding to remoteTimeTics,
   * identically to the return value of a _subsequent_ call of @see getLocalTimestamp(double)
   * remoteTimeTics must strictly increase with every call to this function!
   * @param remoteTimeTics
   * @param localTimeSecs
   * @return localTimeSecs corresponding to remoteTimeTics
   */
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) = 0;

  virtual bool isReadyToTranslate() const = 0;
  virtual void reset() = 0;

  virtual void printNameAndConfig(std::ostream & o) const = 0;
  virtual void printState(std::ostream & o) const = 0;

  std::unique_ptr<OneWayTranslator> clone() const;
 protected:
  virtual OneWayTranslator* cloneImpl() const = 0;
};


/**
 * The receive time pass through One Way Translator does ignore the device time and yields the receive time.
 * This implies that translateToLocalTimestamp is not implemented (throws) because no receive time is available.
 */
class ReceiveTimePassThroughOwt : public OneWayTranslator {
 public:
  virtual ~ReceiveTimePassThroughOwt();
  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) override;
  virtual bool isReadyToTranslate() const override;
  virtual void reset() override;

  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;
 protected:
  virtual ReceiveTimePassThroughOwt* cloneImpl() const override;
};

//TODO cleanup: remove all traces of NopOwt.
typedef ReceiveTimePassThroughOwt NopOwt; // For backwards compatibility

/**
 * The device time pass through One Way Translator does ignore the receive time and yields the device time.
 */
class DeviceTimePassThroughOwt : public OneWayTranslator {
 public:
  virtual ~DeviceTimePassThroughOwt();
  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) override;
  virtual bool isReadyToTranslate() const override;
  virtual void reset() override;

  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;
 protected:
  virtual DeviceTimePassThroughOwt* cloneImpl() const override;
};


} /* namespace cuckoo_time_translator */

#endif /* H3A613172_9ECF_45BC_8D47_C711DE333A50 */
