// Implementation modified from:
// https://github.com/Apollys/cpp-utils/blob/master/misc/timer.hpp

#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

namespace FrictionFrenzy {
class Timer {
	using high_res_clock = std::chrono::high_resolution_clock;

   private:
	double prev_duration_;
	bool running_;
	high_res_clock::time_point start_time_point_;
	high_res_clock::time_point end_time_point_;

	double ComputeDuration(high_res_clock::time_point start,
	                       high_res_clock::time_point end) const {
		using DurationType = std::chrono::duration<double>;
		DurationType dur =
			std::chrono::duration_cast<DurationType>(end - start);
		return dur.count();
	}

   public:
	Timer()
		: prev_duration_(0),
		  running_(false),
		  start_time_point_(high_res_clock::now()) {}

	// Resets and starts timer
	void Reset() {
		prev_duration_ = 0;
		running_ = false;
		start_time_point_ = high_res_clock::now();
	}

	void Pause() {
		end_time_point_ = high_res_clock::now();
		prev_duration_ += ComputeDuration(start_time_point_, end_time_point_);
		running_ = false;
	}

	// Restarts timer without resetting the total duration
	void Resume() {
		running_ = true;
		start_time_point_ = high_res_clock::now();
	}

	double GetSeconds() const {
		if (running_) {
			return prev_duration_ +
			       ComputeDuration(start_time_point_, high_res_clock::now());
		}
		return prev_duration_;
	}
};
}  // namespace FrictionFrenzy

#endif  // TIMER_HPP
