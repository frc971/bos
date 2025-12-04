#include <chrono>
#include <string>
namespace utils {
class Timer {
 public:
  Timer(std::string name);
  ~Timer();
  void Stop();

 private:
  std::string name_;
  bool stopped_;
  std::chrono::system_clock::time_point start_;
};

}  // namespace utils
