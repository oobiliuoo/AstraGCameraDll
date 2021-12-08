#include "pch.h"
#include "BLfactory.h"
#include"AstraGCamera.h"

BLfactory::BLfactory()
{

}

BLfactory::~BLfactory()
{

}

BLAstraCamrea* BLfactory::createBLAstraCamera()
{
	return new AstraGCamera;
}

void BLfactory::deleteObject(BLAstraCamrea* _cam)
{
	if (_cam)
		delete _cam;
}
