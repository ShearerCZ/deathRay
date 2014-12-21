//
//  Phidget.h
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//

#ifndef __dr__Phidget__
#define __dr__Phidget__

#include <phidget21.h>

class PhidgetController
{
    // Whether to report positional changes
    static bool positionalReporting;

    void openManager();
 
    void openServo();
    
    static int CCONV AttachHandler (CPhidgetHandle device, void *userptr);
    
    static int CCONV DetachHandler (CPhidgetHandle device, void *userptr);
    
    static int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle SERV, void *usrptr, int Index, double Value);
    
    // When using an error handler with the manager, it takes a
    // CPhidgetManagerHandle, when using an individual object, the
    // object serves as its own handle.
    static int CCONV LibraryErrorHandler (CPhidgetManagerHandle device, void *usrptr, int errorCode, const char *errorDescription);
    
    static int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description);
    
    // This error handler can handle any CPhidget function that returns an int
    static int LocalErrorCatcher (int errorCode);

    CPhidgetManagerHandle device;
    
    CPhidgetAdvancedServoHandle servo;
    
    //Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
    void display_properties(CPhidgetAdvancedServoHandle phid);
    
    void releaseServo();
    
    void releaseManager();

    public:
        // Sets whether to report positional changes
        static void SetPositionalReporting(bool enabled);
    
        // Constructor
        PhidgetController();
    
        // Get current position of the server
        double GetPosition(unsigned char index) const;
    
        // Set engaged state
        void SetEngaged(unsigned char index, bool value) const;
    
        // Issue next position
        void SetPosition(unsigned char index, double position) const;
    
        // Destructor
        ~PhidgetController();
};


#endif /* defined(__dr__Phidget__) */
