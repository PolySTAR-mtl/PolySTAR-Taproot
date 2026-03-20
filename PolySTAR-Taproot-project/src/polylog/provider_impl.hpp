#ifndef PROVIDER_IMPL_HPP
#define PROVIDER_IMPL_HPP

#include <optional>
#include <functional>

#include "provider.hpp"

namespace polylog
{

template<typename T>
void Provider<T>::provide(T* instance) 
{
    instance_ = instance;
}


template<typename T>
std::optional<std::reference_wrapper<T>> Provider<T>::tryGet() 
{
    if (instance_ == nullptr)
    {
        return std::nullopt;
    }

    return *instance_;
}

}

#endif  // PROVIDER_IMPL_HPP