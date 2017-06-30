#ifndef HD5867872_5846_4710_82B2_BDFD92BC89EC
#define HD5867872_5846_4710_82B2_BDFD92BC89EC

#include <vector>
#include <cstdint>

#include "AbstractAssert.h"
#include "OneWayTranslator.h"

namespace cuckoo_time_translator {

/**
 * \class ConvexHull
 *
 * An implementation of the convex hull algorithm for one-way
 * timestamp synchronization from
 *
 * L. Zhang, Z. Liu, and C. Honghui Xia,
 * “Clock synchronization algorithms for network measurements”,
 * in INFOCOM 2002. Twenty-First Annual Joint Conference of the
 * IEEE Computer and Communications Societies., vol. 1. IEEE,
 * 2002, pp. 160–169 vol.1.
 *
 */
template<typename TIME_T>
class ConvexHull {
 public:
  typedef TIME_T time_t;

  ConvexHull();
  virtual ~ConvexHull();

  /**
   * Get an estimate of the local time of a given measurement
   * from the remote timestamp, the local timestamp, and the
   * previous history of timings.
   *
   * NOTE: this function must be called with monotonically increasing
   *       remote timestamps. If this is not followed, an exception will
   *       be thrown.
   *
   * @param remoteTime The time of an event on the remote clock
   * @param localTime  The timestamp that the event was received locally
   *
   * @return The estimated actual local time of the event
   */
  time_t correctTimestamp(const time_t & remoteTime, const time_t & localTime);

  /**
   * Using the current best estimate of the relationship between
   * remote and local clocks, get the local time of a remote timestamp.
   *
   * @param remoteTime The time of an event on the remote clock.
   *
   * @return The estimated local time of the event.
   */
  time_t getLocalTime(const time_t & remoteTime) const;

  /**
   * @return The number of points in the convex hull
   */
  std::size_t convexHullSize() const {
    return _convexHull.size();
  }

  /**
   * @return The timespan of the convex hull
   */
  time_t span() const;

  /**
   * Clear the points of the convex hull
   */
  void reset() { _convexHull.clear(); _midpointSegmentIndex = 0; }

  double getSlope() const;
  double getOffset() const;

  void printHullPoints(std::ostream & o) const;

 private:
  class Point;

  /**
   * Is the point above the line defined by the top two points of
   * the convex hull?
   *
   * @param p the point to check
   *
   * @return true if the point is above the line.
   */
  bool isAboveTopLine(const Point& p) const;

  /**
   * Is the point below the line defined by the top two points of
   * the convex hull?
   *
   * @param p the point to check
   *
   * @return true if the point is above the line.
   */
  bool isBelowTopLine(const Point& p) const;

  /**
   * Is the point, p, above the line defined by the points l1 and l2?
   *
   * @param l1
   * @param l2
   * @param p
   *
   * @return
   */
  bool isAboveLine(const Point& l1, const Point& l2, const Point& p) const;

  typedef std::vector<Point> convex_hull_t;
  convex_hull_t _convexHull;

  size_t _midpointSegmentIndex;
};

extern template class ConvexHull<double>;
extern template class ConvexHull<std::int64_t>;
extern template class ConvexHull<std::uint64_t>;

}  // namespace cuckoo_time_translator

#endif /* HD5867872_5846_4710_82B2_BDFD92BC89EC */
