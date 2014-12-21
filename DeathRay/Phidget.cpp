//
//  Phidget.cpp
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//
#include "stdafx.h"
#include "Phidget.h"
#include <stdio.h>

bool PhidgetController::positionalReporting(true);

// -------------------- Event Functions ---------------------------------------

int CCONV PhidgetController::AttachHandler (CPhidgetHandle device, void *userptr)
{
    
    int serialNumber;
    const char *name;
    
    LocalErrorCatcher(CPhidget_getDeviceName(device, &name));
    LocalErrorCatcher(CPhidget_getSerialNumber(device, &serialNumber));
    
    printf("%s %10d attached!\n", name, serialNumber);
    
    return 0;
}

int CCONV PhidgetController::DetachHandler (CPhidgetHandle device, void *userptr)
{
    
    int serialNumber;
    const char * name;
    
    LocalErrorCatcher(CPhidget_getDeviceName(device, &name));
    LocalErrorCatcher(CPhidget_getSerialNumber(device, &serialNumber));
    
    printf("%s %10d detached!\n", name, serialNumber);
    
    return 0;
}

// When using an error handler with the manager, it takes a
// CPhidgetManagerHandle, when using an individual object, the
// object serves as its own handle.
int CCONV PhidgetController::LibraryErrorHandler (CPhidgetManagerHandle device, void *usrptr, int errorCode, const char *errorDescription)
{
    printf("Error Event: %d - %s\n", errorCode, errorDescription);
    return 0;
}

int CCONV PhidgetController::ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
    printf("Error handled. %d - %s\n", ErrorCode, Description);
    return 0;
}

int CCONV PhidgetController::PositionChangeHandler(CPhidgetAdvancedServoHandle SERV, void *usrptr, int Index, double Value)
{
    if (positionalReporting)
        printf("Motor: %d > Current Position: %f\n", Index, Value);
    return 0;
}

// This error handler can handle any CPhidget function that returns an int
int PhidgetController::LocalErrorCatcher (int errorCode)
{
    const char *errorDescription;
    
    // If the error code is 0, everything is okay
    if (errorCode)
    {
        // Otherwise, you can print specific messages or perform actions by error value.
        switch (errorCode)
        {
            default:
                printf("Error: An error occurred with code %d.\n", errorCode);
                LocalErrorCatcher(CPhidget_getErrorDescription(errorCode, &errorDescription));
                printf("The description for this error is: %s\n", errorDescription);
                break;
        }
    }
    return 0;
}

void PhidgetController::openManager()
{
    printf("Opening manager\n");
    
    LocalErrorCatcher(CPhidgetManager_create(&device));
    
    LocalErrorCatcher(CPhidgetManager_set_OnAttach_Handler((CPhidgetManagerHandle) device, AttachHandler,       NULL));
    LocalErrorCatcher(CPhidgetManager_set_OnDetach_Handler((CPhidgetManagerHandle) device, DetachHandler,       NULL));
    LocalErrorCatcher(CPhidgetManager_set_OnError_Handler( (CPhidgetManagerHandle) device, LibraryErrorHandler, NULL));
    
    // Most opening and closing would be via a cast to
    // (CPhidgetHandle), however, this manager has its
    // own handle struct to cast to.
    LocalErrorCatcher(CPhidgetManager_open((CPhidgetManagerHandle) device));
}

void PhidgetController::openServo()
{
    printf("Opening servo/n");
    
    //create the servo object
    LocalErrorCatcher(CPhidgetAdvancedServo_create(&servo));
    
    //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
    LocalErrorCatcher(CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL));
    LocalErrorCatcher(CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL));
    LocalErrorCatcher(CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL));
    
    //Registers a callback that will run when the motor position is changed.
    //Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    LocalErrorCatcher(CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL));
    
    //open the servo for device connections
    LocalErrorCatcher(CPhidget_open((CPhidgetHandle)servo, -1));
    
    //get the program to wait for an servo device to be attached
    int result;
    const char * err(0);
    printf("Waiting for Servo controller to be attached....");
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return;
    }
    
    // Display the properties of the attached servo device
    display_properties(servo);
    
    // Set up some initial acceleration and velocity values
    double acceleration(0);
    CPhidgetAdvancedServo_getAccelerationMax(servo, 0, & acceleration);
    CPhidgetAdvancedServo_setAcceleration(servo, 0, acceleration);
    
    double velocity(0);
    CPhidgetAdvancedServo_getVelocityMax(servo, 0, &velocity);
    CPhidgetAdvancedServo_setVelocityLimit(servo, 0, velocity);
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
void PhidgetController::display_properties(CPhidgetAdvancedServoHandle phid)
{
    int serialNo, version, numMotors;
    const char* ptr;
    
    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
    
    CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);
    
    printf("%s\n", ptr);
    printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);
}

void PhidgetController::releaseServo()
{
    LocalErrorCatcher(CPhidget_close((CPhidgetHandle)servo));
    LocalErrorCatcher(CPhidget_delete((CPhidgetHandle)servo));
}

void PhidgetController::releaseManager()
{
    printf("Releasing manager\n");
    LocalErrorCatcher(CPhidgetManager_close(( CPhidgetManagerHandle) device));
    LocalErrorCatcher(CPhidgetManager_delete((CPhidgetManagerHandle) device));
}

// Sets whether to report positional changes
void PhidgetController::SetPositionalReporting(bool enabled)
{
    positionalReporting = enabled;
}

// Constructor
PhidgetController::PhidgetController() : device(0), servo(0)
{
    printf("Phidget Initializing\n");
    openManager();
    openServo();
    printf("Phidget Initialized\n");
}

// Get current position of the servo
double PhidgetController::GetPosition(unsigned char index) const
{
    double toRet(0);
    CPhidgetAdvancedServo_getPosition(servo, index, &toRet);
    return toRet;
}

// Set engaged state
void PhidgetController::SetEngaged(unsigned char index, bool value) const
{
    CPhidgetAdvancedServo_setEngaged(servo, index, (value ? 1 : 0));
}

// Issue next position
void PhidgetController::SetPosition(unsigned char index, double position) const
{
    CPhidgetAdvancedServo_setPosition (servo, index, position);
}

// Destructor
PhidgetController::~PhidgetController()
{
    printf("Phidget Closing\n");
    releaseServo();
    releaseManager();
    printf("Phidget Closed\n");
}