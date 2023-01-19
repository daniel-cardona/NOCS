/*
 *
 * Copyright (C) 2020
 * Daniel Cardona-Ortiz <daniel.cardona@cinvestav.edu.mx>, Gustavo Arechavaleta <garechav@cinvestav.edu.mx>
 * CINVESTAV - Saltillo Campus
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include <stdio.h>
#include <stdlib.h>


#include "extApi.h"
#include "extApiPlatform.h"
#include "simConst.h"



int main(int argc,char* argv[])
{
    int clientID=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (clientID!=-1)
    {
        printf("Connected to remote API server\n");

        // Now try to retrieve data in a blocking fashion (i.e. a service call):
        int objectCount;
        int* objectHandles;
        int ret=simxGetObjects(clientID,sim_handle_all,&objectCount,&objectHandles,simx_opmode_blocking);
        if (ret==simx_return_ok)
            printf("Number of objects in the scene: %d\n",objectCount);
        else
            printf("Remote API function call returned with error code: %d\n",ret);

        extApi_sleepMs(2000);

        // Now retrieve streaming data (i.e. in a non-blocking fashion):
        int startTime=extApi_getTimeInMs();
        int mouseX;
        simxGetIntegerParameter(clientID,sim_intparam_mouse_x,&mouseX,simx_opmode_streaming); // Initialize streaming
        while (extApi_getTimeDiffInMs(startTime) < 5000)
        {
            ret=simxGetIntegerParameter(clientID,sim_intparam_mouse_x,&mouseX,simx_opmode_buffer); // Try to retrieve the streamed data
            if (ret==simx_return_ok) // After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                printf("Mouse position x: %d\n",mouseX); // Mouse position x is actualized when the cursor is over CoppeliaSim's window
        }

        // Now send some data to CoppeliaSim in a non-blocking fashion:
        simxAddStatusbarMessage(clientID,"Hello CoppeliaSim!",simx_opmode_oneshot);

        // Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        int pingTime;
        simxGetPingTime(clientID,&pingTime);

        // Now close the connection to CoppeliaSim:
        simxFinish(clientID);
    }
    return(0);
}
