#pragma once
#include "pch.h"
#include "BLAstraGCamera.h"
class CLASS_DECLSPEC BLfactory
{

public:
	BLfactory();
	~BLfactory();

public:
	BLAstraCamrea* createBLAstraCamera();
	void deleteObject(BLAstraCamrea* _cam);

};

