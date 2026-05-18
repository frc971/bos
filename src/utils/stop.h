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

void SignalHandler(int signal) {
  LOG(INFO) << "Received signal: " << signal;
  stop = true;
}

void RegisterHandler() {
  if (registered_handler) {
    LOG(WARNING) << "Handler has already been registred";
    return;
  }
  std::signal(SIGINT, SignalHandler);
  // std::signal(SIGILL, SignalHandler);
  // std::signal(SIGABRT, SignalHandler);
  // std::signal(SIGFPE, SignalHandler);
  // std::signal(SIGSEGV, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  std::signal(SIGHUP, SignalHandler);
  std::signal(SIGQUIT, SignalHandler);
  // std::signal(SIGTRAP, SignalHander);
  // std::signal(SIGKILL, SignalHandler);
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
