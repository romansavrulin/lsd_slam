
#include "util/Parse.h"

#include "ParseArgs.h"

#include <boost/filesystem.hpp>



namespace lsd_slam {

  using std::vector;
  using std::string;

  namespace fs = boost::filesystem;

  ParseArgs::ParseArgs( int argc, char **argv )
    : dataSource( nullptr ),
      undistorter( nullptr )
  {

    std::string calibFile;

    if(Parse::arg(argc, argv, "-c", calibFile) > 0)
    {
      undistorter.reset( libvideoio::UndistorterFactory::getUndistorterForFile(calibFile) );
    } else {
      printf("Need to specify calibration file with -c option");
      exit(0);
    }

    CHECK( undistorter != NULL ) << "Could not create undistorter.";

    // open image files: first try to open as file.
    std::string source;
    if(!(Parse::arg(argc, argv, "-f", source) > 0))
    {
      printf("need source files! (set using -f FOLDER or KLG)\n");
      exit(0);
    }

    std::vector<fs::path> files;

    if( getdir(source, files) >= 0)
    {
      printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
    }
    else
    {
      printf("could not load file list! wrong path / file?\n");
    }

    printf("Loading images from %s\n", source.c_str());
    dataSource.reset(  new libvideoio::ImageFilesSource( files ) );


  }


}
