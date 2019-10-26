#ifndef ARRC_RASPI_TIME_HPP
#define ARRC_RASPI_TIME_HPP
#include <chrono>

using namespace arrc_raspi {
  class Timer {
  public:
    Timer() { reset(); }
    void reset() { prev = std::chrono::high_resolution_clock::now(); }
    void update() {
      now = std::chrono::high_resolution_clock::now();
      duration_time =
          (long double)std::chrono::duration_cast<std::chrono::nanoseconds>(
              now - prev)
              .count() *
          1.0e-9;
      prev = now;
    }
    long double read() { return duration_time; }
    bool sleep(long double wait_time) {
      return duration_time > wait_time ? true : false;
    }

  private:
    std::chrono::high_resolution_clock::time_point now, prev;
    long double duration_time;
    bool timer_stop = false;
  }
}
#endif
