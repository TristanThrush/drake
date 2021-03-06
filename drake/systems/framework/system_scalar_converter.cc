#include "drake/systems/framework/system_scalar_converter.h"

#include "drake/common/hash.h"

using std::pair;
using std::type_index;
using std::type_info;

namespace drake {
namespace systems {

SystemScalarConverter::Key::Key(
    const type_info& t_info, const type_info& u_info)
    : pair<type_index, type_index>(t_info, u_info) {}

size_t SystemScalarConverter::KeyHasher::operator()(const Key& key) const {
  return hash_combine(size_t{}, key.first, key.second);
}

SystemScalarConverter::SystemScalarConverter() = default;

void SystemScalarConverter::Insert(
    const std::type_info& t_info, const std::type_info& u_info,
    const ErasedConverterFunc& converter) {
  const auto& key = Key{t_info, u_info};
  const auto& insert_result = funcs_.insert({key, converter});
  DRAKE_ASSERT(insert_result.second);
}

const SystemScalarConverter::ErasedConverterFunc* SystemScalarConverter::Find(
    const std::type_info& t_info, const std::type_info& u_info) const {
  const auto& key = Key{t_info, u_info};
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    return &(iter->second);
  } else {
    return nullptr;
  }
}

}  // namespace systems
}  // namespace drake
