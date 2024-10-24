#ifndef __RNG_H__
#define __RNG_H__

#include <Common/InputParse.h>
#include <Magnum/Magnum.h>
#include <yaml-cpp/yaml.h>

#include <random>

namespace FrictionFrenzy {
using Magnum::Float;

template <typename T>
class RNG1DBase {
   public:
	RNG1DBase() {}
	virtual ~RNG1DBase() {}
	RNG1DBase(int seed) : m_randomSeed(seed) {}
	RNG1DBase(T avg, int seed) : m_randomSeed(seed), m_avg(avg) {}
	T getAvg() { return m_avg; }
	virtual T sample() = 0;

   protected:
	int m_randomSeed;
	T m_avg;
};

class ConstFloat : public RNG1DBase<Float> {
   public:
	ConstFloat(const YAML::Node& node) {
		m_avg = parseScalar<Float>(node, "value", "ConstFloat");
	}
	Float sample() override { return m_avg; }
};

class UniformFloat : public RNG1DBase<Float> {
   public:
	UniformFloat(Float min, Float max, int seed) {
		m_avg = 0.5 * (min + max);
		m_randomSeed = seed;
		m_gen = std::mt19937(m_randomSeed);
		m_rng = std::uniform_real_distribution<Float>(min, max);
		m_gen.seed(m_randomSeed);
	}
	UniformFloat(const YAML::Node& node) {
		m_avg = parseScalar<Float>(node, "average", "UniformFloat");
		m_randomSeed =
			parseScalar<int>(node, "random_seed", time(NULL), "UniformFloat");
		Float variation = parseScalar<Float>(node, "variation", "UniformFloat");
		m_gen = std::mt19937(m_randomSeed);
		m_rng = std::uniform_real_distribution<Float>(m_avg - variation,
		                                              m_avg + variation);
		m_gen.seed(m_randomSeed);
	}
	Float sample() override { return m_rng(m_gen); }

   protected:
	std::mt19937 m_gen;
	std::uniform_real_distribution<Float> m_rng;
};

class GaussianFloat : public RNG1DBase<Float> {
   public:
	GaussianFloat(Float avg, Float std, Float clamp, int seed)
		: RNG1DBase<Float>(avg, seed), m_std(std), m_clamp(clamp) {}
	Float sample() override {
		Float s;
		do {
			s = m_rng(m_gen);
		} while (m_clamp > 0 && (s < m_avg - m_clamp || s > m_avg + m_clamp));
		// if (m_clamp >= 0) {
		//     s = Magnum::Math::clamp(s, m_avg - m_clamp, m_avg + m_clamp);
		// }
		return s;
	}

   protected:
	std::mt19937 m_gen;
	std::normal_distribution<Float> m_rng;
	Float m_std;
	Float m_clamp;
};

}  // namespace FrictionFrenzy

#endif
