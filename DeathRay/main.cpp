//
//  main.cpp
//  dr
//
//  Created by Hackathon on 18/12/14.
//  Copyright (c) 2014 Hackathon. All rights reserved.
//

/*
 * Phidget Hello World Program for all devices
 * (c) Phidgets 2012
 */


#include <stdio.h>
#include "ServoCtrl.h"

// -------------------- Main Code ---------------------------------------------

void callMe()
{
    PhidgetController pctrl;
    pctrl.SetEngaged(6, true);
    
    double val;
    while (scanf("%lf",&val) == 1)
    {
        printf("Moving to: %lf\n",val);
        pctrl.SetPosition(6, val);
    }
}

void callMe2()
{
    PhidgetController pctrl;
    ServoCtrl ctrl(pctrl, 2, 3);
    
    double const moveStep(3);
    
    while (1)
    {
        char chr;
        scanf(" %c",&chr);
        switch (chr)
        {
                case 'w':
                    ctrl.MoveByServoPos(0, -moveStep);
                    break;
                
                case 's':
                    ctrl.MoveByServoPos(0, moveStep);
                    break;
                
                case 'a':
                    ctrl.MoveByServoPos(-moveStep, 0);
                    break;
                
                case 'd':
                    ctrl.MoveByServoPos(moveStep, 0);
                    break;
                
                case 'q':
                    return;
        }
    }
}

double abs(double val)
{
    if (val >= 0)
        return val;
    else return -val;
}

double max(double a, double b)
{
    if (a > b)
        return a;
    else
        return b;
}

void callMe3()
{
    double const posX[] = { 88.0,  149.7, 151.5,  87.6};
    double const posY[] = { 79.1,  121.4,  80.6, 120.2};
    unsigned char now(0);
    double maxMove(1);
    
    PhidgetController pctrl;
    ServoCtrl ctrl(pctrl, 2, 3);
    
    unsigned tries(10);
    while (--tries)
    {
        ++now;
        if (now == sizeof(posX) / sizeof(double))
            now = 0;
        
        double horizontal;
        double vertical;
        ctrl.GetServoPos(horizontal, vertical);
        double moveHoriz(posX[now] - horizontal);
        double moveVerti(posY[now] - vertical);
        
        while ((int)(moveHoriz * 100) ||  (int)(moveVerti * 100))
        {
            
            double divisor(max(abs(moveHoriz), abs(moveVerti)) / maxMove);
            if (divisor < 1)
                divisor = 1;
            
            ctrl.MoveByServoPos(moveHoriz / divisor, moveVerti / divisor);
            
            ctrl.GetServoPos(horizontal, vertical);
            moveHoriz = posX[now] - horizontal;
            moveVerti = posY[now] - vertical;
        }
    }
}

int main(int argc, char* argv[])
{
    callMe3();
}

