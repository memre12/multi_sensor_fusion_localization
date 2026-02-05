//
// Created by mehce on 16.06.2022.
//

#ifndef NTRIPLIB_THREAD_RAII_H_
#define NTRIPLIB_THREAD_RAII_H_

#include <thread>


namespace libntrip {

class Thread {
 public:
  Thread() = default;
  Thread(std::thread&& t) : thread_(std::move(t)) {}
  Thread(Thread const&) = delete;
  Thread(Thread&&) = delete;
  Thread& operator=(Thread const&) = delete;
  Thread& operator=(Thread&&) = delete;
  template<typename Callable, typename... Args>
  Thread(Callable&& callable, Args&&... args) : Thread(std::thread(
      std::forward<Callable>(callable), std::forward<Args>(args)...)) {}
  ~Thread() { join(); }

  template<typename Callable, typename... Args>
  Thread& reset(Callable&& callable, Args&&... args) {
    join();
    thread_ = std::move(std::thread(
        std::forward<Callable>(callable), std::forward<Args>(args)...));
    return *this;
  }
  Thread& reset(std::thread&& t) {
    join();
    thread_ = std::move(t);
    return *this;
  }
  void join(void) {
    if (thread_.joinable()) thread_.join();
  }
  void detach(void) {
    if (thread_.joinable()) thread_.detach();
  }

 private:
  std::thread thread_;
};

}  // namespace libntrip

#endif  // NTRIPLIB_THREAD_RAII_H_
