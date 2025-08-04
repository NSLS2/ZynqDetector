#pragma once

#include <concepts>

template <typename T, typename X>
concept IsSameType = std::same_as<T, X>;

template <typename T, typename X, typename Y>
concept IsEitherType = IsSameType<T, X>
                    || IsSameType<T, Y>;
