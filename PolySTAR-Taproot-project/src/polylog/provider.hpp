#ifndef PROVIDER_HPP
#define PROVIDER_HPP

#include <optional>
#include <functional>

namespace polylog
{

enum class ProviderError
{
    NoInstanceProvided
};

template<typename T>
class Provider
{
public:
    static void provide(T* instance);

    static std::optional<std::reference_wrapper<T>> tryGet();

private:
    static inline T* instance_ = nullptr;
};

}  // namespace polylog

#include "provider_impl.hpp"

#endif  // PROVIDER_HPP