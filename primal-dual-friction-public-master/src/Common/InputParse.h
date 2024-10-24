#ifndef __INPUT_PARSE_H__
#define __INPUT_PARSE_H__

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

namespace FrictionFrenzy {
class Configurable {
   public:
	virtual void fillFromYAML(const YAML::Node& node) = 0;
};

template <typename T>
inline T parseScalar(const YAML::Node& node,
                     const char* param,
                     const char* structName) {
	if (node[param] && node[param].IsScalar()) {
		return node[param].as<T>();
	} else {
		throw std::runtime_error(std::string(structName) +
		                         ": No value read for parameter '" + param +
		                         "'!");
	}
}

template <typename T>
inline T parseScalar(const YAML::Node& node,
                     const char* param,
                     const T& defValue,
                     const char* structName) {
	if (node[param] && node[param].IsScalar()) {
		return node[param].as<T>();
	} else {
		return defValue;
	}
}

template <typename T, unsigned int n>
Eigen::Vector<T, n> parseVectorEigen(const YAML::Node& node,
                                     const char* param,
                                     const char* structName) {
	if (node[param].IsDefined() && node[param].IsSequence()) {
		Eigen::Vector<T, n> ret;
		for (unsigned int i = 0; i < n; ++i) {
			if (node[param][i].IsDefined() && node[param][i].IsScalar()) {
				ret[i] = node[param][i].as<T>();
			} else {
				throw std::runtime_error(std::string(structName) +
				                         ": invalid input for vector '" +
				                         param + "'!");
			}
		}
		return ret;
	} else {
		throw std::runtime_error(std::string(structName) + ": '" + param +
		                         "' not defined or not a sequence!");
	}
}
}  // namespace FrictionFrenzy

// template<>
// struct YAML::convert<Magnum::Vector2> {
//     static Node encode
// };
#endif
