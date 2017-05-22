#ifndef HD5867872_5846_4710_82B2_BDFD92BC89EB
#define HD5867872_5846_4710_82B2_BDFD92BC89EB

#include "ConvexHull.h"

namespace cuckoo_time_translator {

template<typename T>
ConvexHull<T>::ConvexHull()
    : _midpointSegmentIndex(0u) {
}

template<typename T>
ConvexHull<T>::~ConvexHull() {
}

template<typename T>
class ConvexHull<T>::Point {
 public:
  Point(const time_t & x, const time_t & y) : x(x), y(y) { }
  // remote time
  time_t x;
  // local time
  time_t y;

  Point operator-(const Point& p) const {
    return Point(x - p.x, y - p.y);
  }
  Point operator+(const Point& p) const {
    return Point(x + p.x, y + p.y);
  }
  bool operator<(const Point& p) const {
    return x < p.x;
  }
  bool operator<(const time_t& t) const {
    return x < t;
  }
};

// Returns the local time
template<typename T>
typename ConvexHull<T>::time_t ConvexHull<T>::correctTimestamp(const time_t& remoteTime, const time_t& localTime) {
  // Make sure this point is forward in time.
  if (!_convexHull.empty()) {
    AASSERT_GT(remoteTime, _convexHull.back().x, "This algorithm requires that times are passed in with monotonically increasing remote timestamps");
  }

  const Point p(remoteTime, localTime);

  // If the point is not above the top line in the stack
  if (_convexHull.size() >= 2u && !isAboveTopLine(p)) {
    // While on the top of the stack points are above a line between two back and the new point...
    while (_convexHull.size() >= 2u && isBelowTopLine(p)) {
      _convexHull.pop_back();
    }
  }

  // In either case, push the new point on to the convex hull
  _convexHull.push_back(p);

  // Update the midpoint pointer...
  if (_convexHull.size() >= 3u) {
    T midpoint = static_cast<T>((_convexHull[0u].x + remoteTime) / 2.0);

    typename convex_hull_t::iterator lbit = std::lower_bound(_convexHull.begin(), _convexHull.end(), midpoint);
    _midpointSegmentIndex = lbit - _convexHull.begin() - 1u;
    AASSERT_LT_DBG(_midpointSegmentIndex, _convexHull.size() - 1u, "The computed midpoint segment is out of bounds.");
    AASSERT_GE_DBG(midpoint, _convexHull[_midpointSegmentIndex].x, "The computed midpoint is not within the midpoint segment");
    AASSERT_LE_DBG(midpoint, _convexHull[_midpointSegmentIndex + 1u].x, "The computed midpoint is not within the midpoint segment");
  } else {
    // and if there aren't enough data points, just return the sampled local time.
    return localTime;
  }

  return getLocalTime(remoteTime);

}

template<typename T>
double ConvexHull<T>::getSlope() const {
  AASSERT_GE(_convexHull.size(), 2u, "At least two data points are required before this function can be called");

  // Get the line at the time midpoint.
  const Point& l1 = _convexHull[_midpointSegmentIndex];
  const Point& l2 = _convexHull[_midpointSegmentIndex + 1u];

  // Look up the local timestamp.
  return double(l2.y - l1.y) / double(l2.x - l1.x);
}

template<typename T>
double ConvexHull<T>::getOffset() const {
  AASSERT_GE(_convexHull.size(), 2u, "At least two data points are required before this function can be called");
  // Get the line at the time midpoint.
  const Point& l1 = _convexHull[_midpointSegmentIndex];
  const Point& l2 = _convexHull[_midpointSegmentIndex + 1u];

  // Look up the local timestamp.
  return double(l1.y) + (double(-l1.x) * double(l2.y - l1.y) / double(l2.x - l1.x));
}

template<typename T>
void ConvexHull<T>::printHullPoints(std::ostream & o) const {
  for (unsigned i = 0u; i < _convexHull.size(); ++i) {
    o << i << "\t" << _convexHull[i].x << "\t" << _convexHull[i].y;
    if (i == _midpointSegmentIndex)
      o << " <<< Midpoint segment start";
    o << std::endl;
  }
}

template<typename T>
typename ConvexHull<T>::time_t ConvexHull<T>::span() const
{
  return this->convexHullSize() > 2 ? (_convexHull.back().y - _convexHull.front().y) : static_cast<time_t>(0);
}
// Get the local time from the remote time.
template<typename T>
typename ConvexHull<T>::time_t ConvexHull<T>::getLocalTime(const time_t& remoteTime) const {
  AASSERT_GE(_convexHull.size(), 2u, "The timestamp correction requires at least two data points before this function can be called");

  // Get the line at the time midpoint.
  const Point& l1 = _convexHull[_midpointSegmentIndex];
  const Point& l2 = _convexHull[_midpointSegmentIndex + 1u];

  // Look up the local timestamp.
  const double helper = static_cast<double>(l2.y - l1.y) / static_cast<double>(l2.x - l1.x);
  const time_t localTimeSecs = static_cast<time_t>(static_cast<double>(l1.y) + helper * static_cast<double>(remoteTime - l1.x));
  return localTimeSecs;
}

template<typename T>
bool ConvexHull<T>::isAboveTopLine(const Point& p) const {
  return isAboveLine(_convexHull[_convexHull.size() - 2u], _convexHull[_convexHull.size() - 1u], p);
}
template<typename T>
bool ConvexHull<T>::isBelowTopLine(const Point& p) const {
  return isAboveLine(_convexHull[_convexHull.size() - 2u], p, _convexHull[_convexHull.size() - 1u]);
}

template<typename T>
bool ConvexHull<T>::isAboveLine(const Point & l1, const Point & l2, const Point & p) const {
  const Point v1 = l2 - l1;
  const Point v2 = p - l1;

  const T determinant = v1.x * v2.y - v1.y * v2.x;

  return determinant >= static_cast<T>(0.0);
}



}  // namespace cuckoo_time_translator

#endif /* HD5867872_5846_4710_82B2_BDFD92BC89EB */
