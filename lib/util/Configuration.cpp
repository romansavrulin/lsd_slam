#include "Configuration.h"

namespace lsd_slam {

  Configuration::Configuration() :
      doDepth( NO_STEREO ),
      stopOnFailedRead( true ),
      SLAMEnabled( true ),
      doKFReActivation( true ),
      doMapping( true ),
      continuousPCOutput( false ),

      autoRun( true ),
      autoRunWithinFrame( true ),

      debugDisplay( 0 ),

      onSceenInfoDisplay( true ),
      displayDepthMap( true ),
      dumpMap( false ),
      doFullReConstraintTrack( false )
  {;}

}
