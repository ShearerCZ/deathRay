//
//  CameraServo.h
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//

#ifndef __dr__CameraServo__
#define __dr__CameraServo__

#include <iostream>

struct CameraServoMapping
{
    double camera[2];
    double servo[2];
    
    CameraServoMapping(double cameraX, double cameraY, double servoX, double servoY);
    
    CameraServoMapping(CameraServoMapping const & copy);
    
    CameraServoMapping(CameraServoMapping && mve);
    
    CameraServoMapping(std::istream & stream);
    
    bool operator==(CameraServoMapping const & other);
    
    CameraServoMapping & operator=(CameraServoMapping const & other);
    
    void Serialize(std::ostream & stream) const;
};

#endif /* defined(__dr__CameraServo__) */
