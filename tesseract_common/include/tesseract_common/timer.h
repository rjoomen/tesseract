/**
 * @file timer.h
 * @brief Simple timer class using chrono and thread
 *
 * @author Levi Armstrong
 * @date February 2, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COMMON_TIMER_H
#define TESSERACT_COMMON_TIMER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/** @brief A timer which calls a callback every interval on a separate thread */
class Timer
{
public:
  Timer() = default;
  ~Timer();
  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;
  Timer(Timer&&) = delete;
  Timer& operator=(Timer&&) = delete;

  /**
   * @brief Start the timer with a callback function and a std::chrono::duration interval
   * @param callback The callback called every time the timer expires
   * @param interval The interval at which the timer triggers
   */
  void start(const std::function<void()>& callback, std::chrono::steady_clock::duration interval);

  /** @brief Stop the timer */
  void stop();

private:
  std::atomic<bool> running_{ false };
  std::thread timer_thread_;
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_TIMER_H
