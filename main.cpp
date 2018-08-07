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
		omni.showSavedLabel();

		if (omni._refineStep == 0)
			omni.label();
		else if (omni._refineStep == 1)
			omni.refineMonoLabel();
		else if (omni._refineStep == 2)
			omni.addMissingGlobalBVLabel();
		else if (omni._refineStep == 3)
			omni.refineGlobalBVLabel();

		omni.keyborad();
	}
	omni.release();
	return 0;
}

