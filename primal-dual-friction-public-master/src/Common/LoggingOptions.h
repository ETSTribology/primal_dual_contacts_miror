#pragma once

#include <memory>

namespace FrictionFrenzy {
enum class LoggingOptions {
	MISC_DEBUG = (1 << 0),
	PER_ITER_ERRORS = (1 << 1),
	PER_STEP_ERRORS = (1 << 2),
	PER_STEP_CONTACTS = (1 << 3),
	PER_STEP_TIME = (1 << 4),
	PER_STEP_SPEEDS = (1 << 5),
	PER_STEP_CHARFACTOR = (1 << 6),
	MAX_PENETRATION_DEPTH = (1 << 7),
	PER_STEP_FRICTION_COEFF = (1 << 8)
};

inline bool hasLoggingOption(std::shared_ptr<unsigned int> opt,
                             LoggingOptions search) {
	return (*opt) & (unsigned int)(search);
}

inline void setLoggingOption(std::shared_ptr<unsigned int>& opt,
                             LoggingOptions setting,
                             bool val) {
	if (val) {
		*opt |= (unsigned int)setting;
	} else {
		*opt &= ~((unsigned int)setting);
	}
}
}  // namespace FrictionFrenzy
