#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer
{
   private:
      std::chrono::high_resolution_clock::time_point start_duration;
      std::chrono::high_resolution_clock::time_point end_duration;

   public:
      Timer();

      void start();
      void end();
      double get_exec_time();
};

#endif