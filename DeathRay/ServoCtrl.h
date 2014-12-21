//
//  ServoCtrl.h
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//

#ifndef __dr__ServoCtrl__
#define __dr__ServoCtrl__

#include "Phidget.h"
#include <vector>
#include "CameraServo.h"

class ServoCtrl
{
    static float const SERVO_OFFSET_TOLERATION;
    
    PhidgetController const & phidget;
    
    unsigned char const horizontalServo;
    
    unsigned char verticalServo;
    
    std::vector<CameraServoMapping> mappings;
    
    double getDistance(CameraServoMapping const & mapping, double cameraPosX, double cameraPosY) const;
    
    void getMappingKoefs(double & koefX, double & koefY) const;
    
public:
    ServoCtrl(PhidgetController const & phidget, unsigned char horizontalServo, unsigned char verticalServo);
    
    // 0 - 220
    void GetServoPos(double & posX, double & posY) const;

    void MoveByServoPos(double horizontal = 1.0, double vertical = 1.0, bool waitForIt = false) const;
    
    // 0 - 220
    void MoveToServoPos(double posX = 100, double posY = 100, bool waitForIt = false) const;
    
    // 0 - 1
    void MoveToCameraPos(double posX = 0.5, double posY = 0.5, bool waitForIt = false) const;
    
    // 0 - 1
    void CalibrateCameraPos(double posX, double posY);
    
    // Save calibration file
    void SaveCalibration(char const * path) const;
    
    // Load calibration file
    void LoadCalibration(char const * path);
    
    ~ServoCtrl();
};

#endif /* defined(__dr__ServoCtrl__) */
