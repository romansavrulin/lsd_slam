#include "Configuration.h"

namespace lsd_slam {

  Configuration::Configuration() :
      runRealTime( true ),
      doDepth( NO_STEREO ),
      stopOnFailedRead( true ),
      SLAMEnabled( false ),
      doKFReActivation( true ),
      doMapping( true ),
      continuousPCOutput( true ),

      autoRun( true ),
      autoRunWithinFrame( true ),

      debugDisplay( 0 ),

      onSceenInfoDisplay( true ),
      displayDepthMap( true ),
      dumpMap( false ),
      doFullReConstraintTrack( false )
  {;}

}
