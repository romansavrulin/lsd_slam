
#include <iostream>


#include "g3logger.h"

#include <g3log/g3log.hpp>


namespace lsd_slam {

G3Logger::G3Logger( const std::string &appName )
  : worker( g3::LogWorker::createLogWorker() ),
  stderrHandle( worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                       &ColorStderrSink::ReceiveLogMessage) )
{

  auto handle = worker->addDefaultLogger(appName, ".");

  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);

  // This should be the only message written explicitly to std::cout
  // Everything else gets sent to the logger
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n*\n" << std::endl;

}


void G3Logger::verbose( bool verbose )
{
   stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
}

void G3Logger::logBanner( void )
{

  LOG(INFO) << "Starting log.";

  #ifdef ENABLE_SSE
    LOG(INFO) << "With SSE optimizations.";
  #elif ENABLE_NEON
    LOG(INFO) << "With NEON optimizations.";
  #endif

}

}
