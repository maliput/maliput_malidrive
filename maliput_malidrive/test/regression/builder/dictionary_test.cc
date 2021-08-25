// Copyright 2020 Toyota Research Institute
#include "maliput_malidrive/builder/dictionary.h"

#include <array>
#include <functional>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {
namespace test {
namespace {

template <typename T, typename U>
bool AreElementsEqual(const T& lhs, const U& rhs) {
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

// Tests the default constructor.
GTEST_TEST(DictionaryTest, DefaultConstructor) {
  const Dictionary<int, int> dut;
  ASSERT_EQ(0, dut.Keys().size());
}

// Tests the constructor with a predefined bucket size (capacity).
GTEST_TEST(DictionaryTest, BucketSizeConstructor) {
  constexpr std::size_t kBucketSize{10};

  const Dictionary<int, int> dut(kBucketSize);
  ASSERT_EQ(kBucketSize, dut.Keys().capacity());
}

// Tests that the dut emplaces the value.
GTEST_TEST(DictionaryTest, EmplaceInEmptyDictionary) {
  constexpr int kKey{1};
  constexpr int kValue{2};

  Dictionary<int, int> dut;
  ASSERT_TRUE(dut.Emplace(kKey, kValue));
  ASSERT_EQ(kValue, dut.at(kKey));
}

// Fails to emplace many times the same key.
GTEST_TEST(DictionaryTest, EmplaceTwiceTheSameValue) {
  constexpr int kKey{1};
  constexpr int kValueA{2};
  constexpr int kValueB{2};

  Dictionary<int, int> dut;
  ASSERT_TRUE(dut.Emplace(kKey, kValueA));
  ASSERT_FALSE(dut.Emplace(kKey, kValueB));
}

// Evaluates the underlying data.
GTEST_TEST(DictionaryTest, Values) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValues{2, 6, 4, 8};
  const std::unordered_map<int, int> kExpectedDict{{1, 2}, {3, 6}, {2, 4}, {4, 8}};

  Dictionary<int, int> dut;
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }

  ASSERT_EQ(kExpectedDict, dut.Values());
}

// Evaluates the unsorted keys.
GTEST_TEST(DictionaryTest, EvaluateUnsortedKeys) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValues{2, 6, 4, 8};

  Dictionary<int, int> dut;
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }

  ASSERT_TRUE(AreElementsEqual(kKeys, dut.Keys()));
}

// Evaluates the default sorting algorithm keys.
GTEST_TEST(DictionaryTest, EvaluateDefaultKeys) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kSortedKeys{1, 2, 3, 4};
  const std::array<int, 4> kValues{2, 6, 4, 8};

  Dictionary<int, int> dut;
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }

  dut.SortKeys();
  ASSERT_TRUE(AreElementsEqual(kSortedKeys, dut.Keys()));
}

// Evaluates a custom sorting algorithm.
GTEST_TEST(DictionaryTest, EvaluateDescendingKeys) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kSortedKeys{4, 3, 2, 1};
  const std::array<int, 4> kValues{2, 6, 4, 8};

  Dictionary<int, int> dut;
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }

  dut.SortKeys(std::greater<int>());
  ASSERT_TRUE(AreElementsEqual(kSortedKeys, dut.Keys()));
}

// Fills and clears the collection.
GTEST_TEST(DictionaryTest, Clear) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValues{2, 6, 4, 8};

  Dictionary<int, int> dut;
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }
  ASSERT_EQ(kKeys.size(), dut.Keys().size());

  dut.Clear();
  ASSERT_TRUE(dut.Keys().empty());
}

// Evaluates the collection capacity.
GTEST_TEST(DictionaryTest, Reserve) {
  constexpr std::size_t kBucketSize{10};
  constexpr std::size_t kDoubleBucketSize{2 * kBucketSize};
  constexpr std::size_t kHalfBucketSize{kBucketSize / 2};

  // Creates a dictionary with a certain capacity.
  Dictionary<int, int> dut(kBucketSize);
  ASSERT_EQ(kBucketSize, dut.Keys().capacity());

  // Increases capacity.
  dut.Reserve(kDoubleBucketSize);
  ASSERT_EQ(kDoubleBucketSize, dut.Keys().capacity());

  // Tries to reduce it, but the previous allocation remains.
  dut.Reserve(kHalfBucketSize);
  ASSERT_EQ(kDoubleBucketSize, dut.Keys().capacity());
}

// Tests that the values are correctly placed and could be accessed via keys.
GTEST_TEST(DictionaryTest, ConstAtAccess) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValues{2, 6, 4, 8};

  Dictionary<int, int> dut;
  // Initializes the dictionary.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValues[i]));
  }

  // Evaluates its contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_EQ(kValues[i], dut.at(kKeys[i]));
  }
}

// Tests that the values are correctly placed and could be accessed via keys.
GTEST_TEST(DictionaryTest, MutableAtAccess) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValuesA{2, 6, 4, 8};
  const std::array<int, 4> kValuesB{3, 9, 6, 12};

  Dictionary<int, int> dut;
  // Initializes the dictionary.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], kValuesA[i]));
  }

  // Evaluates the contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_EQ(kValuesA[i], dut.at(kKeys[i]));
  }

  // Updates the contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    dut.at(kKeys[i]) = kValuesB[i];
  }

  // Evaluates the new contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_EQ(kValuesB[i], dut.at(kKeys[i]));
  }
}

class NoMoveNoCopyNoAssignType {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(NoMoveNoCopyNoAssignType);

  explicit NoMoveNoCopyNoAssignType(int data) : data_(data) {}

  int data() const { return data_; }

  bool operator==(const NoMoveNoCopyNoAssignType& rhs) const { return data_ == rhs.data_; }

 private:
  int data_{};
};

// Tests that the values are correctly placed and could be accessed via keys.
// Using std::unique_ptr<> makes the Dictionary::Emplace() operation actually
// move references rather than creating copies.
GTEST_TEST(DictionaryTest, ComplexType) {
  const std::array<int, 4> kKeys{1, 3, 2, 4};
  const std::array<int, 4> kValuesA{2, 6, 4, 8};
  const std::array<int, 4> kValuesB{3, 9, 6, 12};

  Dictionary<int, std::unique_ptr<NoMoveNoCopyNoAssignType>> dut;

  // Initializes the dictionary.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_TRUE(dut.Emplace(kKeys[i], std::make_unique<NoMoveNoCopyNoAssignType>(kValuesA[i])));
  }

  // Evaluates the contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_EQ(kValuesA[i], dut.at(kKeys[i])->data());
  }

  // Updates the contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    dut.at(kKeys[i]) = std::make_unique<NoMoveNoCopyNoAssignType>(kValuesB[i]);
  }

  // Evaluates the new contents.
  for (std::size_t i = 0; i < kKeys.size(); ++i) {
    ASSERT_EQ(kValuesB[i], dut.at(kKeys[i])->data());
  }
}


}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace malidrive
