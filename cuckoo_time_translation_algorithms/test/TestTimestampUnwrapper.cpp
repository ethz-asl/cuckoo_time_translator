#include <cuckoo_time_translator/TimestampUnwrapper.h>

#include <gtest/gtest.h>

using namespace cuckoo_time_translator;

class CaptureStream {
  std::stringstream buffer;
  std::streambuf *oldBuffer;
  std::ostream & ostream_;
public:
  CaptureStream(std::ostream & ostream) :
    ostream_(ostream)
  {
    oldBuffer = std::cout.rdbuf();
    ostream_.rdbuf(buffer.rdbuf());
  }

  ~CaptureStream(){
    ostream_.rdbuf(oldBuffer);
  }

  std::string getAndFlush() {
    auto val = buffer.str();
    buffer.str(std::string());
    return val;
  }
};

TEST(TimeUnwrapper, Init) {
  const uint64_t kWrapAroundNumber = 1000ul;
  TimestampUnwrapper unwrapper(kWrapAroundNumber, 100u);

  EXPECT_EQ(0u, unwrapper.getUnwrappedStamp().getValue());
  EXPECT_EQ(kWrapAroundNumber, unwrapper.getWrapAroundNumber());

  EXPECT_EQ(0.0, unwrapper.toSec());
  EXPECT_EQ(0.0, unwrapper.toSec(unwrapper.getUnwrappedStamp()));
}

TEST(TimeUnwrapper, Wrap) {
  TimestampUnwrapper unwrapper(1000ul, 100u);

  unwrapper.updateWithNewStamp(900u);
  EXPECT_EQ(900u, unwrapper.getUnwrappedStamp().getValue());
  EXPECT_EQ(9.0, unwrapper.toSec());
  EXPECT_EQ(9.0, unwrapper.toSec(unwrapper.getUnwrappedStamp()));

  unwrapper.updateWithNewStamp(0u);
  EXPECT_EQ(1000lu, unwrapper.getUnwrappedStamp().getValue());
  EXPECT_EQ(10.0, unwrapper.toSec());
  EXPECT_EQ(10.0, unwrapper.toSec(unwrapper.getUnwrappedStamp()));
}

TEST(TimeUnwrapper, detectWrapAroundNumberTooSmall) {
  TimestampUnwrapper unwrapper(1000ul, 100u);
  CaptureStream captureErr(std::cerr);

  unwrapper.updateWithNewStamp(1999u);
  EXPECT_EQ(2000lu, unwrapper.getWrapAroundNumber());
  std::string expectedOutput = "Error:   newStamp=1999 is larger than wrapAroundNumber=1000 -> adapting wrapAroundNumber to 2000!";
  EXPECT_EQ(expectedOutput, captureErr.getAndFlush().substr(0, expectedOutput.size()));
  unwrapper.updateWithNewStamp(10u);
  EXPECT_EQ(2000lu + 10lu, unwrapper.getUnwrappedStamp().getValue());
}

TEST(TimeUnwrapper, detectWrapAroundNumberTooLarge) {
  TimestampUnwrapper unwrapper(1000ul, 100u);
  CaptureStream captureErr(std::cerr);

  for (int i = 0; i < 10; i ++){
    unwrapper.updateWithNewStamp(500u);
    unwrapper.updateWithNewStamp(0u);
  }
  std::string expectedOutput = "Warning: Last maxStamp=500, suspiciously small! Maybe it wraps in fact earlier? (wrapAroundNumber=1000)";
  EXPECT_EQ(expectedOutput, captureErr.getAndFlush().substr(0, expectedOutput.size()));
}
