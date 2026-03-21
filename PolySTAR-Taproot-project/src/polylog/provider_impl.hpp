#ifndef PROVIDER_IMPL_HPP
#define PROVIDER_IMPL_HPP

#include "provider.hpp"

namespace polylog
{

template <typename T>
void Provider<T>::provide(T* instance)
{
    instance_ = instance;
}

template <typename T>
[[nodiscard]] T* Provider<T>::tryGet()
{
    return instance_;
}

}  // namespace polylog

#endif  // PROVIDER_IMPL_HPP