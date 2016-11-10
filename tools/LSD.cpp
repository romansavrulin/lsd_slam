/**
*
* Based on original LSD-SLAM code from:
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include "util/G3LogSinks.h"

#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Configuration.h"
#include "util/FileUtils.h"

#include "LSD/LSD.h"


using namespace lsd_slam;

ThreadMutexObject<bool> lsdDone(false), guiDone(false);

ThreadSynchronizer lsdReady, guiReady, startAll;

int main( int argc, char** argv )
{
  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(argv[0], ".");
  auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                       &ColorStderrSink::ReceiveLogMessage);

  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n\n" << std::endl;

  LOG(INFO) << "Starting log.";

  DataSource *dataSource = NULL;
  Undistorter* undistorter = NULL;

#ifdef ENABLE_SSE
  LOG(INFO) << "With SSE optimizations.";
#elif ENABLE_NEON
  LOG(INFO) << "With NEON optimizations.";
#endif

  Configuration conf;

  bool doGui = true;

  std::string calibFile;

  if(Parse::arg(argc, argv, "-c", calibFile) > 0)
  {
     undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
  }
  // open image files: first try to open as file.
	std::string source;
	if(!(Parse::arg(argc, argv, "-f", source) > 0))
	{
		printf("need source files! (set using -f FOLDER or KLG)\n");
		exit(0);
	}

  std::vector<std::string> files;

  if(getdir(source, files) >= 0)
  {
      printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
  }
  else
  {
      printf("could not load file list! wrong path / file?\n");
  }

  printf("Loading images from %s\n", source.c_str());
  dataSource = new ImagesSource( files );


  CHECK( undistorter != NULL ) << "Could not create undistorter.";
  CHECK( dataSource != NULL ) << "Could not create data source.";

  // Load the configuration object

  conf.inputImage = undistorter->inputImageSize();
  conf.slamImage  = undistorter->outputImageSize();
  conf.camera     = undistorter->getCamera();

  LOG(INFO) << "Slam image: " << conf.slamImage.width << " x " << conf.slamImage.height;

  CHECK( (conf.camera.fx) > 0 && (conf.camera.fy > 0) ) << "Camera focal length is zero";

	SlamSystem * system = new SlamSystem(conf);

  if( doGui ) {
    LOG(INFO) << "Starting GUI thread";
    boost::thread guiThread(runGui, system );
    guiReady.wait();
  }

  LOG(INFO) << "Starting input thread.";
  boost::thread inputThread(runInput, system, dataSource, undistorter );
  lsdReady.wait();

  // Wait for all threads to be ready.
  LOG(INFO) << "Starting all threads.";
  startAll.notify();

  while(true)
  {
      if( (lsdDone.getValue() || guiDone.getValue()) && !system->finalized)
      {
          LOG(INFO) << "Finalizing system.";
          system->finalize();
      }

    sleep(1);
  }


  if( system ) delete system;
  if( undistorter ) delete undistorter;

  return 0;
}
