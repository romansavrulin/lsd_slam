
#include <iostream>


#include "g3logger.h"

#include <g3log/g3log.hpp>
#include "G3LogSinks.h"

namespace lsd_slam {

std::unique_ptr<g3::LogWorker> initializeG3Log( const std::string &appName )
{
  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(appName, ".");

  // Uses a custom log sing (ColorStderrSink)
  auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                       &ColorStderrSink::ReceiveLogMessage);

  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);

  // This should be the only message written explicitly to std::cout
  // Everything else gets sent to the logger
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n*\n" << std::endl;

  return worker;
}


void logBanner( void )
{

  LOG(INFO) << "Starting log.";

  #ifdef ENABLE_SSE
    LOG(INFO) << "With SSE optimizations.";
  #elif ENABLE_NEON
    LOG(INFO) << "With NEON optimizations.";
  #endif

}

}
