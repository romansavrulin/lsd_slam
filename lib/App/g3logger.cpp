

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>


namespace lsd_slam {



void initializeG3Logger( void )
{
  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(argv[0], ".");
  auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                       &ColorStderrSink::ReceiveLogMessage);

  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);

  // This should be the only message written explicitly to std::cout
  // Everything else gets logged
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n*\n" << std::endl;
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
