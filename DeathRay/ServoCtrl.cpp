//
//  ServoCtrl.cpp
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//
#include "ServoCtrl.h"
#include <fstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

float const ServoCtrl::SERVO_OFFSET_TOLERATION(0.05);

ServoCtrl::ServoCtrl(PhidgetController const & phidget, unsigned char horizontalServo, unsigned char verticalServo)
	: phidget(phidget), horizontalServo(horizontalServo), verticalServo(verticalServo)
{
	phidget.SetEngaged(horizontalServo, true);
	phidget.SetEngaged(verticalServo, true);
}

double ServoCtrl::getDistance(CameraServoMapping const & mapping, double cameraPosX, double cameraPosY) const
{
	return (mapping.camera[0] - cameraPosX)*(mapping.camera[0] - cameraPosX) + (mapping.camera[1] - cameraPosY)*(mapping.camera[1] - cameraPosY);
}

void ServoCtrl::getMappingKoefs(double & koefX, double & koefY) const
{
	double cameraLow[2] = {mappings.front().camera[0], mappings.front().camera[1]};
	double cameraHigh[2] = {mappings.front().camera[0], mappings.front().camera[1]};
	double servoLow[2] = {mappings.front().servo[0], mappings.front().servo[1]};
	double servoHigh[2] = {mappings.front().servo[0], mappings.front().servo[1]};
	
	for (auto pnter(mappings.cbegin()); pnter!=mappings.cend(); ++pnter)
	{
		if (pnter->camera[0] < cameraLow[0])
			cameraLow[0] = pnter->camera[0];
		if (pnter->camera[1] < cameraLow[1])
			cameraLow[1] = pnter->camera[1];
		
		if (pnter->camera[0] > cameraHigh[0])
			cameraHigh[0] = pnter->camera[0];
		if (pnter->camera[1] > cameraHigh[1])
			cameraHigh[1] = pnter->camera[1];
		
		if (pnter->servo[0] < servoLow[0])
			servoLow[0] = pnter->servo[0];
		if (pnter->servo[1] < servoLow[1])
			servoLow[1] = pnter->servo[1];
		
		if (pnter->servo[0] > servoHigh[0])
			servoHigh[0] = pnter->servo[0];
		if (pnter->servo[1] > servoHigh[1])
			servoHigh[1] = pnter->servo[1];
	}
	
	koefX =(servoHigh[0] - servoLow[0]) / (cameraHigh[0] - cameraLow[0]);
	koefY =(servoHigh[1] - servoLow[1]) / (cameraHigh[1] - cameraLow[1]);
}

// 0 - 220
void ServoCtrl::GetServoPos(double & posX, double & posY) const
{
	posX = phidget.GetPosition(horizontalServo);
	posY = phidget.GetPosition(verticalServo);
}

void ServoCtrl::MoveByServoPos(double horizontal, double vertical, bool waitForIt) const
{
	auto posX(phidget.GetPosition(horizontalServo) + horizontal);
	auto posY(phidget.GetPosition(verticalServo) + vertical);
	
	MoveToServoPos(posX, posY, waitForIt);
}

// 0 - 220
void ServoCtrl::MoveToServoPos(double posX, double posY, bool waitForIt) const
{
	phidget.SetPosition(horizontalServo, posX);
	phidget.SetPosition(verticalServo, posY);
	
	double realX;
	double realY;
	GetServoPos(realX, realY);
	
	/*
	while (std::abs(realX - posX) + std::abs(realY - posY) > SERVO_OFFSET_TOLERATION)
	{
		phidget.SetPosition(horizontalServo, posX);
		phidget.SetPosition(verticalServo, posY);
		GetServoPos(realX, realY);
	}
	*/
}

// 0 - 1
void ServoCtrl::MoveToCameraPos(double posX, double posY, bool waitForIt) const
{
	// Need at least two calibrations to work (better more)
	if (mappings.size() < 2)
	{
		MoveByServoPos();
		return;
	}
	
	// Get closest candidate
	auto closest(mappings.cbegin());
	auto closestDst(getDistance(*closest, posX, posY));
	for (auto pnter(mappings.cbegin()); pnter!=mappings.cend(); ++pnter)
	{
		auto nowDst(getDistance(*pnter, posX, posY));
		if (nowDst < closestDst)
		{
			closest = pnter;
			closestDst = nowDst;
		}
	}
	
	// koefs
	double koefX;
	double koefY;
	getMappingKoefs(koefX, koefY);

	auto targetX(closest->servo[0] + (posX - closest->camera[0])*koefX);
	auto targetY(closest->servo[1] + (posY - closest->camera[1])*koefY);
	
	MoveToServoPos(targetX, targetY, waitForIt);
}

// 0 - 1
void ServoCtrl::CalibrateCameraPos(double posX, double posY)
{
	CameraServoMapping newMapping(posX, posY, phidget.GetPosition(horizontalServo), phidget.GetPosition(verticalServo));
	//cout << "Calibration mapping: X: " << posX << " Y: " << posY;
	// Update if same mapping appears twice
	for (auto pnter(mappings.begin()); pnter!=mappings.end(); ++pnter)
	{
		if (*pnter == newMapping)
		{
			*pnter = newMapping;
			return;
		}
	}
	
	// Add if new one
	mappings.push_back(newMapping);
}

// Save calibration file
void ServoCtrl::SaveCalibration(char const * path) const
{
	std::fstream file(path, std::ios::out | std::ios::trunc | std::ios::binary);
	
	for (auto pnter(mappings.cbegin()); pnter!=mappings.cend(); ++pnter)
		pnter->Serialize(file);
	
	file.close();
}

// Load calibration file
void ServoCtrl::LoadCalibration(char const * path)
{
	mappings.clear();
	std::fstream file(path, std::ios::in | std::ios::binary);
	
	while (file.good())
	{
		CameraServoMapping loadedMapping(file);
		if (file.good())
			mappings.push_back(std::move(loadedMapping));
	}
	
	file.close();
}

ServoCtrl::~ServoCtrl()
{
	phidget.SetEngaged(horizontalServo, true);
	phidget.SetEngaged(verticalServo, true);
}