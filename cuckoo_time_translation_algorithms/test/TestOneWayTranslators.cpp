#include <cuckoo_time_translator/ConvexHullOwt.h>
#include <cuckoo_time_translator/SwitchingOwt.h>

#include <gtest/gtest.h>

using namespace cuckoo_time_translator;


typedef std::vector<TimePair> VTP;
typedef std::vector<double> VD;

template <typename Translator>
VD feedData(Translator && owt, const VTP & data){
  VD result;
  for(auto & tp : data){
    result.emplace_back(tp.update(owt));
  }
  return result;
}

TEST(ConvexHull, testConvexHullOwt) {
  ConvexHullOwt chOwt;
  auto app = [&](const VTP & data){return feedData(chOwt, data); };
  auto getSizeOffsetAndSkew = [&](){return VD{double(chOwt.getStackSize()), chOwt.getOffset(), chOwt.getSkew()}; };

  EXPECT_EQ(VD({0., 1.}), app({ {0._R, 0._L}, {1._R, 1._L} }));
  EXPECT_EQ(VD({2, 0., 1.}), getSizeOffsetAndSkew());
  EXPECT_EQ(VD({2.}), app({ {2._R, 2.5_L} }));
  EXPECT_EQ(VD({3, 0., 1.}), getSizeOffsetAndSkew());

  chOwt.reset();

  EXPECT_EQ(VD({0., 1., 2., 3.}), app({ {0._R, 0._L}, {1._R, 1._L}, {2._R, 2.5_L}, {3._R, 3._L} }));
  EXPECT_EQ(VD({2, 0., 1.}), getSizeOffsetAndSkew());

  chOwt.reset();

  ASSERT_EQ(VD({0, 1., 1.5, 1.5/2.*3.}), app({ {0._R, 0._L}, {1._R, 1._L}, {2._R, 1.5_L}, {3._R, 3._L} }));
  EXPECT_EQ(VD({3, 0., 1.5/2.}), getSizeOffsetAndSkew());

  chOwt.reset();

  EXPECT_EQ(0, chOwt.getStackSize());
}


TEST(ConvexHull, testSwitchingOwt) {
  SwitchingOwt swOwt = SwitchingOwt::craeteSwitchingOwt<ConvexHullOwt>(2.0);

  auto app = [&](const VTP & data){return feedData(swOwt, data); };
  auto getSwitchCountTimeAndSizes = [&](){return VD{
      double(swOwt.getSwitchCount()),
      double(swOwt.getLastSwitchTime()),
      double(static_cast<ConvexHullOwt&>(swOwt.getCurrentOwt()).getStackSize()),
      double(static_cast<ConvexHullOwt&>(swOwt.getPendingOwt()).getStackSize()) };
  };

  EXPECT_EQ(VD({0, -1, 0, 0}), getSwitchCountTimeAndSizes());

  EXPECT_EQ(VD({0., 1.}), app({ {0._R, 0._L}, {1._R, 1._L} })); // <- new last switch time == 0.0
  EXPECT_EQ(VD({0, 0, 2, 2}), getSwitchCountTimeAndSizes());

  EXPECT_EQ(VD({2.}), app({ {2._R, 2._L} })); // <- new last switch time == 2.0
  EXPECT_EQ(VD({1, 2, 3, 1}), getSwitchCountTimeAndSizes());

  EXPECT_EQ(VD({3.}), app({ {3._R, 3._L} }));
  EXPECT_EQ(VD({1, 2, 4, 2}), getSwitchCountTimeAndSizes());

  EXPECT_EQ(VD({4.}), app({ {4._R, 4._L} })); // <- new last switch time == 4.0
  EXPECT_EQ(VD({2, 4, 3, 1}), getSwitchCountTimeAndSizes());

  EXPECT_EQ(VD({5.}), app({ {5._R, 5._L} }));
  EXPECT_EQ(VD({2, 4, 4, 2}), getSwitchCountTimeAndSizes());

  swOwt.reset();

  EXPECT_EQ(VD({0, -1, 0, 0}), getSwitchCountTimeAndSizes());
}
