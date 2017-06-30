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
  TimestampUnwrapperEventOnly unwrapper({kWrapAroundNumber, 100u});

  EXPECT_EQ(0u, unwrapper.getUnwrappedEventStamp().getValue());
  EXPECT_EQ(kWrapAroundNumber, unwrapper.getWrapAroundNumber());

  EXPECT_EQ(0.0, unwrapper.getEventStampSec());
  EXPECT_EQ(0.0, unwrapper.stampToSec(unwrapper.getUnwrappedEventStamp()));
}

TEST(TimeUnwrapper, Wrap) {
  TimestampUnwrapperEventOnly unwrapper({1000ul, 100u});

  unwrapper.updateWithNewEventStamp(900u);
  EXPECT_EQ(900u, unwrapper.getUnwrappedEventStamp().getValue());
  EXPECT_EQ(9.0, unwrapper.getEventStampSec());
  EXPECT_EQ(9.0, unwrapper.stampToSec(unwrapper.getUnwrappedEventStamp()));

  unwrapper.updateWithNewEventStamp(0u);
  EXPECT_EQ(1000lu, unwrapper.getUnwrappedEventStamp().getValue());
  EXPECT_EQ(10.0, unwrapper.getEventStampSec());
  EXPECT_EQ(10.0, unwrapper.stampToSec(unwrapper.getUnwrappedEventStamp()));
}

TEST(TimeUnwrapper, WrapWithTransmit) {
  TimestampUnwrapperEventAndTransmit unwrapper({1000ul, 100u});

  unwrapper.updateWithNewEventStamp(900u);
  unwrapper.updateWithNewTransmitStamp(950u);
  EXPECT_EQ(900u, unwrapper.getUnwrappedEventStamp().getValue());
  EXPECT_EQ(950u, unwrapper.getUnwrappedTransmitStamp().getValue());
  EXPECT_EQ(9.0, unwrapper.getEventStampSec());
  EXPECT_EQ(9.0, unwrapper.stampToSec(unwrapper.getUnwrappedEventStamp()));
  EXPECT_EQ(9.5, unwrapper.getTransmitStampSec());
  EXPECT_EQ(9.5, unwrapper.stampToSec(unwrapper.getUnwrappedTransmitStamp()));

  unwrapper.updateWithNewEventStamp(0u);
  unwrapper.updateWithNewTransmitStamp(50u);
  EXPECT_EQ(1000lu, unwrapper.getUnwrappedEventStamp().getValue());
  EXPECT_EQ(10.0, unwrapper.getEventStampSec());
  EXPECT_EQ(10.0, unwrapper.stampToSec(unwrapper.getUnwrappedEventStamp()));
  EXPECT_EQ(1050lu, unwrapper.getUnwrappedTransmitStamp().getValue());
  EXPECT_EQ(10.5, unwrapper.getTransmitStampSec());
  EXPECT_EQ(10.5, unwrapper.stampToSec(unwrapper.getUnwrappedTransmitStamp()));
}

TEST(TimeUnwrapper, detectWrapAroundNumberTooSmall) {
  TimestampUnwrapperEventOnly unwrapper({1000ul, 100u});
  CaptureStream captureErr(std::cerr);

  unwrapper.updateWithNewEventStamp(1999u);
  EXPECT_EQ(2000lu, unwrapper.getWrapAroundNumber());
  std::string expectedOutput = "Error:   newDeviceStamp=1999 is larger than wrapAroundNumber=1000 -> adapting wrapAroundNumber to 2000!";
  EXPECT_EQ(expectedOutput, captureErr.getAndFlush().substr(0, expectedOutput.size()));
  unwrapper.updateWithNewEventStamp(10u);
  EXPECT_EQ(2000lu + 10lu, unwrapper.getUnwrappedEventStamp().getValue());
}

TEST(TimeUnwrapper, detectWrapAroundNumberTooLarge) {
  TimestampUnwrapperEventOnly unwrapper({1000ul, 100u});
  CaptureStream captureErr(std::cerr);

  for (int i = 0; i < 10; i ++){
    unwrapper.updateWithNewEventStamp(500u);
    unwrapper.updateWithNewEventStamp(0u);
  }
  std::string expectedOutput = "Warning: Last maxStamp=500, suspiciously small! Maybe it wraps in fact earlier? (wrapAroundNumber=1000)";
  EXPECT_EQ(expectedOutput, captureErr.getAndFlush().substr(0, expectedOutput.size()));
}
