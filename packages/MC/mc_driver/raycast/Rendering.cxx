//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2014 Sandia Corporation.
//  Copyright 2014 UT-Battelle, LLC.
//  Copyright 2014 Los Alamos National Security.
//
//  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
//  the U.S. Government retains certain rights in this software.
//
//  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
//  Laboratory (LANL), the U.S. Government retains certain rights in
//  this software.
//============================================================================

//We first check if VTKM_DEVICE_ADAPTER is defined, so that when TBB and CUDA
//includes this file we use the device adapter that they have set.
#ifndef VTKM_DEVICE_ADAPTER
#define VTKM_DEVICE_ADAPTER VTKM_DEVICE_ADAPTER_SERIAL
#endif

#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/testing/MakeTestDataSet.h>

//Suppress warnings about glut being deprecated on OSX
#if (defined(VTKM_GCC) || defined(VTKM_CLANG))
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <vtkm/rendering/internal/OpenGLHeaders.h> //Required for compile....

#if defined (__APPLE__)
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif

#include <vtkm/rendering/CanvasRayTracer.h>
#include "MapperRayTracer.h"
#include <vtkm/rendering/View3D.h>
#include <vtkm/rendering/ColorTable.h>

#include <QRgb>
#include <QImage>
#include <QImageWriter>

vtkm::rendering::View3D *view = nullptr;

const vtkm::Int32 W = 512, H = 512;
int buttonStates[3] = {GLUT_UP, GLUT_UP, GLUT_UP};
bool shiftKey = false;
int lastx=-1, lasty=-1;

void
reshape(int, int)
{
    //Don't allow resizing window.
    glutReshapeWindow(W,H);
}

// Render the output using simple OpenGL
void displayCall()
{
    view->Paint();
    glutSwapBuffers();
}

// Allow rotations of the camera
void mouseMove(int x, int y)
{
    //std::cout<<"MOUSE MOVE: "<<x<<" "<<y<<std::endl;

    const vtkm::Id width = view->GetCanvas().GetWidth();
    const vtkm::Id height = view->GetCanvas().GetHeight();

    //Map to XY
    y = static_cast<int>(height-y);

    if (lastx != -1 && lasty != -1)
    {
        vtkm::Float32 x1 = ((lastx*2.0f)/width) - 1.0f;
        vtkm::Float32 y1 = ((lasty*2.0f)/height) - 1.0f;
        vtkm::Float32 x2 = ((x*2.0f)/width) - 1.0f;
        vtkm::Float32 y2 = ((y*2.0f)/height) - 1.0f;

        if (buttonStates[0] == GLUT_DOWN)
        {
            if (shiftKey)
                view->GetCamera().Pan(x2-x1, y2-y1);
            else
                view->GetCamera().TrackballRotate(x1,y1, x2,y2);
        }
        else if (buttonStates[1] == GLUT_DOWN)
            view->GetCamera().Zoom(y2-y1);
    }

    lastx = x;
    lasty = y;
    glutPostRedisplay();
}


// Respond to mouse button
void mouseCall(int button, int state, int vtkmNotUsed(x), int vtkmNotUsed(y))
{
    int modifiers = glutGetModifiers();
    shiftKey = modifiers & GLUT_ACTIVE_SHIFT;
    buttonStates[button] = state;

    //std::cout<<"Buttons: "<<buttonStates[0]<<" "<<buttonStates[1]<<" "<<buttonStates[2]<<" SHIFT= "<<shiftKey<<std::endl;

    //mouse down, reset.
    if (buttonStates[button] == GLUT_DOWN)
    {
        lastx = -1;
        lasty = -1;
    }
}

// Compute and render an isosurface for a uniform grid example
int
main(int argc, char* argv[])
{



    return 0;
}

#if (defined(VTKM_GCC) || defined(VTKM_CLANG))
# pragma GCC diagnostic pop
#endif
