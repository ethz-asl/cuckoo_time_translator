#ifndef H2C3CEBCE_5B75_4AA3_B9F5_1513B4FA2EFD
#define H2C3CEBCE_5B75_4AA3_B9F5_1513B4FA2EFD

namespace cuckoo_time_translator {
class OneWayTranslator;

class TaggedTime {
 public:
  explicit TaggedTime(double v) : value(v) {}
  operator double () const { return value; }

  bool operator == (const TaggedTime& other) const { return value == other.value; }
  bool operator != (const TaggedTime& other) const { return value != other.value; }

 private:
  double value;
};

class RemoteTime : public TaggedTime {
  using TaggedTime::TaggedTime;
};
class LocalTime : public TaggedTime {
  using TaggedTime::TaggedTime;
};

inline LocalTime operator "" _L(long double t) {
  return LocalTime(t);
}
inline RemoteTime operator "" _R(long double t) {
  return RemoteTime(t);
}

struct TimePair {
  RemoteTime remote;
  LocalTime local;

  double update(OneWayTranslator & owt) const;
};


} // namespace cuckoo_time_translator

#endif /* H2C3CEBCE_5B75_4AA3_B9F5_1513B4FA2EFD */
