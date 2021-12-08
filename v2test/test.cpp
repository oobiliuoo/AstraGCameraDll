#include "test.h"


void test1()
{

	BLfactory b;
	BLAstraCamrea* cam = b.createBLAstraCamera();
	cam->setShowMode(3);
	cam->start();

	cam->close();
	b.deleteObject(cam);

}