#ifndef PROVIDER_HPP
#define PROVIDER_HPP

namespace polylog
{

enum class ProviderError
{
    NoInstanceProvided
};

template <typename T>
class Provider
{
public:
    static void provide(T* instance);

    [[nodiscard]] static T* tryGet();

private:
    static inline T* instance_ = nullptr;
};

}  // namespace polylog

#include "provider_impl.hpp"

#endif  // PROVIDER_HPP