#pragma once

#include <concepts>

// 定义 IsSameAs 用于检查类型是否与 X 相同
template <typename T, typename X>
concept IsSameType = std::same_as<T, X>;

template <typename T, typename X, typename Y>
concept IsEitherType = IsSameType<T, X>
                    || IsSameType<T, Y>;