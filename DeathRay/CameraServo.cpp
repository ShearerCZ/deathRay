//
//  CameraServo.cpp
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//
#include "CameraServo.h"
#include <string.h>
#include <cmath>

CameraServoMapping::CameraServoMapping(double cameraX, double cameraY, double servoX, double servoY)
{
    camera[0] = cameraX;
    camera[1] = cameraY;
    servo[0] = servoX;
    servo[1] = servoY;
}

CameraServoMapping::CameraServoMapping(CameraServoMapping const & copy)
{
    memcpy(this, &copy, sizeof(*this));
}

CameraServoMapping::CameraServoMapping(CameraServoMapping && mve)
{
    memcpy(this, &mve, sizeof(*this));
}

CameraServoMapping & CameraServoMapping::operator=(CameraServoMapping const & other)
{
    memcpy(this, &other, sizeof(*this));
    return *this;
}

CameraServoMapping::CameraServoMapping(std::istream & stream)
{
    stream >> camera[0];
    stream >> camera[1];
    stream >> servo[0];
    stream >> servo[1];
}

bool CameraServoMapping::operator==(CameraServoMapping const & other)
{
    return camera[0] == other.camera[0] && camera[1] == other.camera[1];
}

void CameraServoMapping::Serialize(std::ostream & stream) const
{
    stream << camera[0];
    stream << camera[1];
    stream << servo[0];
    stream << servo[1];
}
