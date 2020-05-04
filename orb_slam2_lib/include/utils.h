#pragma once

//
// Just a few utility functions. This file should be kept small.
//

#include <memory>


// A (partial) backport of C++14's `make_unique` to C++11.
// Does not support the case of T being an array.
template <typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>{ new T(std::forward<Args>(args)...) };
}

