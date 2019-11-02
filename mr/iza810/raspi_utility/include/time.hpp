#ifndef ARRC_RASPI_TIME_HPP
#define ARRC_RASPI_TIME_HPP
#include <chrono>
#include <thread>

namespace arrc_raspi {
class Timer {
public:
  Timer() { reset(); }
  void reset() { start = prev = std::chrono::high_resolution_clock::now(); }
  void update() {
    now = std::chrono::high_resolution_clock::now();
    duration_time =
        (long double)std::chrono::duration_cast<std::chrono::nanoseconds>(now -
                                                                          prev)
            .count() *
        1.0e-9;
    elapsed_time =
        (long double)std::chrono::duration_cast<std::chrono::nanoseconds>(now -
                                                                          start)
            .count() *
        1.0e-9;
    prev = now;
  }
  long double read() { return duration_time; }
  bool wait(long double wait_time) {
    return elapsed_time > wait_time ? true : false;
  }
  void sleep(long double sleep_time) {
    std::this_thread::sleep_for(
        std::chrono::microseconds((int)(sleep_time * 1.0e+6)));
  }

private:
  std::chrono::high_resolution_clock::time_point now, prev, start;
  long double duration_time, elapsed_time;
  bool timer_stop = false;
};
} // namespace arrc_raspi
#endif
