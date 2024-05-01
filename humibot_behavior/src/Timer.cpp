#include "Timer.hpp"

Timer::Timer()
{  
}

void Timer::start()
{
   this->start_duration = std::chrono::high_resolution_clock::now();
}
void Timer::end()
{
   this->end_duration = std::chrono::high_resolution_clock::now();
}
double Timer::get_exec_time()
{
   std::chrono::duration<double, std::milli> duration = this->end_duration - this->start_duration;
   return duration.count();
}