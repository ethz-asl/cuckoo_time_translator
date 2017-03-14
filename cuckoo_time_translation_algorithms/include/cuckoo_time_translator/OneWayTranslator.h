#ifndef H3A613172_9ECF_45BC_8D47_C711DE333A50
#define H3A613172_9ECF_45BC_8D47_C711DE333A50

#include <iosfwd>

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

  virtual bool isReady() const = 0;

  virtual void printNameAndConfig(std::ostream & o) const = 0;
  virtual void printState(std::ostream & o) const = 0;
  virtual void reset() = 0;
};

/**
 * The No Operation One Way Translator does no translation at all.
 * I.e. it passes the receive time through.
 */
class NopOwt : public OneWayTranslator {
 public:
  virtual ~NopOwt();
  virtual LocalTime translateToLocalTimestamp(RemoteTime remoteTimeTics) const override;
  virtual LocalTime updateAndTranslateToLocalTimestamp(RemoteTime remoteTimeTics, LocalTime localTimeSecs) override;
  virtual bool isReady() const override;
  virtual void printNameAndConfig(std::ostream & o) const override;
  virtual void printState(std::ostream & o) const override;
  virtual void reset() override;
};

} /* namespace cuckoo_time_translator */

#endif /* H3A613172_9ECF_45BC_8D47_C711DE333A50 */
