#ifndef __HASHING_H__
#define __HASHING_H__
//
//  this is from boost
//

// #include <Magnum/Magnum.h>
#include <functional>

template <typename T>
inline void hash_combine(std::size_t& seed, const T& val) {
	std::hash<T> hasher;
	seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// template <typename T>
// struct std::hash<Magnum::Math::Vector2<T>> {
// 	size_t operator()(const Magnum::Math::Vector2<T>& vec) const {
// 		size_t seed = 0;
// 		hash_combine(seed, vec[0]);
// 		hash_combine(seed, vec[1]);
// 		return seed;
// 	}
// };
template <typename T, typename S>
struct std::hash<std::pair<T, S>> {
	size_t operator()(const std::pair<T, S>& vec) const {
		size_t seed = 0;
		hash_combine(seed, vec.first);
		hash_combine(seed, vec.second);
		return seed;
	}
};
#endif
