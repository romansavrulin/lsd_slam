
#include "LSD.h"

#include "Pangolin_IOWrapper/PangolinOutput3DWrapper.h"

using namespace lsd_slam;

void runGuiThread(const std::shared_ptr<GUI> &gui )
{
	guiReady.notify();
	startAll.wait();

	while(!pangolin::ShouldQuit())
	{
		if(guiDone.getValue()) break;

		LOG(INFO) << "runGuiThread";

		gui->preCall();
		gui->drawKeyframes();

		gui->drawFrustum();

		gui->drawImages();

		gui->postCall();
	}

	guiDone.assignValue(true);

}
