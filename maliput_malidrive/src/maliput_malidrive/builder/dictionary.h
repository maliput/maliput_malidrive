// Copyright 2021 Toyota Research Institute
#pragma once

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

namespace malidrive {
namespace builder {

/// Dictionary backed by a std::unordered_map<Key, Value> and a
/// std::vector<Key>.
///
/// Both the map and the vector could reserve the bucket size, which provides
/// an advantage for collection management when the size is well known.
///
/// Keys could be sorted which allows ordered iteration in constant time rather
/// than logarithmic time as std::map offers.
///
/// This collection is suitable when:
/// - the number of elements to be stored is known in advance, so Reserve() can
///   be used.
/// - usage would consist in filling the collection, and then iterating or
///   accessing by key elements.
///
/// @tparam Key The key of the dictionary. It must satisfy std::unordered_map's
///         key requirements as well as std::sort() compare function.
/// @tparam Value The value to store in the dictionary.
template <typename Key, typename Value>
class Dictionary {
 public:
  Dictionary() = default;

  /// Constructs a dictionary with @p bucket_count elements.
  /// @param bucket_count  The number of elements the dictionary has.
  Dictionary(std::size_t bucket_count) : data_(bucket_count), keys_(bucket_count) {}

  /// Forwards the call to the inner collections to emplace a new value into the
  /// dictionary.
  /// @param args Arguments that will be forwarded to std::unordered_map<Key, Value>::emplace().
  ///             When successful, the result of the operation yields an iterator to the new
  ///             element and its key which is incorporated into the keys container as well.
  /// @return true When the element could be emplaced.
  /// @tparams Args The variadic template argument list which consists of the Key
  ///          and the construction arguments of the element.
  template <class... Args>
  bool Emplace(Args&&... args) {
    const auto result = data_.emplace(std::move(args)...);
    if (result.second) {
      keys_.emplace_back(result.first->first);
    }
    return result.second;
  }

  /// Sorts the keys in descending order.
  void SortKeys() { std::sort(keys_.begin(), keys_.end()); }

  /// Sorts the keys with a custom sorting function.
  template <typename CompareFunction>
  void SortKeys(CompareFunction cmp) {
    std::sort(keys_.begin(), keys_.end(), cmp);
  }

  /// Accesses the keys.
  /// @return A constant reference to the keys vector.
  const std::vector<Key>& Keys() const { return keys_; }

  /// Accesses the keys.
  /// @return A constant reference to the underlying data.
  const std::unordered_map<Key, Value>& Values() const { return data_; }

  /// Clears the dictionary.
  void Clear() {
    data_.clear();
    keys_.clear();
  }

  /// Reserves @p bucket_count elements in the dictionary.
  /// This function forwards the call to std::vector<Key>::reserve() and
  /// std::unordered_map<Key, Value>::reserve(). Should consider its
  /// implications.
  void Reserve(std::size_t bucket_count) {
    data_.reserve(bucket_count);
    keys_.reserve(bucket_count);
  }

  /// Accesses an element of the dictionary.
  /// @param key The key of the element.
  /// @return A constant reference to the value referred by @p key.
  /// @throw std::out_of_range When the key is unknown.
  const Value& at(const Key& key) const { return data_.at(key); }

  /// Accesses an element of the dictionary.
  /// @param key The key of the element.
  /// @return A mutable reference to the value referred by @p key.
  /// @throw std::out_of_range When the key is unknown.
  Value& at(const Key& key) { return data_.at(key); }

 private:
  std::unordered_map<Key, Value> data_{};
  std::vector<Key> keys_{};
};

}  // namespace builder
}  // namespace malidrive
