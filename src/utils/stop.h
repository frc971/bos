#pragma once

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>
#include "src/utils/log.h"

namespace stop {

using namespace std::literals::chrono_literals;

constexpr std::chrono::seconds kwait_interval = 1s;
std::atomic<bool> stop(false);
std::atomic<bool> registered_handler(false);

void SignalHander(int signal) {
  LOG(INFO) << "Received signal: " << signal;
  stop = true;
}

void RegisterHandler() {
  if (registered_handler) {
    LOG(WARNING) << "Handler has already been registred";
    return;
  }
  std::signal(SIGINT, SignalHander);
  std::signal(SIGILL, SignalHander);
  std::signal(SIGABRT, SignalHander);
  std::signal(SIGFPE, SignalHander);
  std::signal(SIGSEGV, SignalHander);
  std::signal(SIGTERM, SignalHander);
  std::signal(SIGHUP, SignalHander);
  std::signal(SIGQUIT, SignalHander);
  // std::signal(SIGTRAP, SignalHander);
  std::signal(SIGKILL, SignalHander);
  // std::signal(SIGPIPE, SignalHander);
  // std::signal(SIGALRM, SignalHander);
  registered_handler = true;
}

void WaitUntilStop() {
  while (!stop) {
    std::this_thread::sleep_for(stop::kwait_interval);
  }
}
}  // namespace stop
