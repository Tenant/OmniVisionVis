#include <iostream>
#include <string>
#include <vector>
#include "OmniVision.h"

using namespace std;
using namespace cv;

int main()
{
	OmniVision omni;
	if (!omni.init())
		return 0;

	while (omni.getData())
	{
		//omni.testImageP2VehicleP();
		//omni.showLMS();
		omni.showVelo();

		if (!omni.keyborad())
			break;
	}
	omni.release();
	return 0;
}