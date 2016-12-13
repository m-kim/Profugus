//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   MC/mc_driver/Manager_VTKm.hh
 * \author Thomas M>. Evans
 * \date   Wed Jun 18 11:21:16 2014
 * \brief  Manager class definition.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef MC_mc_driver_Manager_VTKm_hh
#define MC_mc_driver_Manager_VTKm_hh

//#include <sstream>
//#include <string>
//#include <memory>

//#include "comm/P_Stream.hh"
//#include "mc/Fixed_Source_Solver.hh"
//#include "mc/Keff_Solver.hh"
//#include "mc/Global_RNG.hh"
//#include "mc/Source_Transporter.hh"
//#include "Problem_Builder.hh"
#include "Manager.hh"

#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/testing/MakeTestDataSet.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include "raycast/MapperRayTracer.h"
#include <vtkm/rendering/View3D.h>
#include <vtkm/rendering/ColorTable.h>
//#include <QRgb>
//#include <QImage>
//#include <QImageWriter>


namespace mc{

//===========================================================================//
/*!
 * \class Manager_VTKm
 * \brief Manager_VTKm class that drives the MC miniapp.
 */
//===========================================================================//

template <class Geometry>
class Manager_VTKm : public Manager<Geometry>
{
  public:
    // Constructor.
    Manager_VTKm();

    // Output.
    void output();
private:
    void tmp();

    int lastx, lasty;
    vtkm::rendering::View3D *view;
};



//---------------------------------------------------------------------------//
// CONSTRUCTOR
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
template <class Geometry>
Manager_VTKm<Geometry>::Manager_VTKm()
    : Manager<Geometry>()
{
  lastx = lasty = -1;
  view = nullptr;
}


//---------------------------------------------------------------------------//
/*!
 * \brief Do output.
 */
template <class Geometry>
void Manager_VTKm<Geometry>::output()
{
  Manager<Geometry>::output();

  std::cout << "WOOT" << std::endl;
  tmp();
}

template <class Geometry>
void Manager_VTKm<Geometry>::tmp()
{
  vtkm::cont::DataSetBuilderExplicitIterative dataSetBuilder;

  //cylinder
  dataSetBuilder.AddPoint(1, 0.25, 1);
  dataSetBuilder.AddPoint(1, .75, 1);

  //box
  dataSetBuilder.AddPoint(0.25, 0.25, 0.25);
  dataSetBuilder.AddPoint(0.75, .75, 0.75);

  //sphere
  dataSetBuilder.AddPoint(1.5, 0.5, 1);

  dataSetBuilder.AddCell(vtkm::CELL_SHAPE_TRIANGLE);
  dataSetBuilder.AddCellPoint(0);
  dataSetBuilder.AddCellPoint(1);

  dataSetBuilder.AddCell(vtkm::CELL_SHAPE_LINE);
  dataSetBuilder.AddCellPoint(2);
  dataSetBuilder.AddCellPoint(3);

  dataSetBuilder.AddCell(vtkm::CELL_SHAPE_VERTEX);
  dataSetBuilder.AddCellPoint(4);

  vtkm::cont::DataSet csg = dataSetBuilder.Create();

  vtkm::cont::DataSetFieldAdd dataSetFieldAdd;

  std::vector<vtkm::Float32> radii(5);
  //cylinder
  radii[0] = 0.2;
  radii[1] = 0.2;
  //box
  radii[2] = 0;
  radii[3] = 0;
  //sphere
  radii[4] = 0.1;

  dataSetFieldAdd.AddPointField(csg, "radius", radii);

  lastx = lasty = -1;

//    glutInit(&argc, argv);
//    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
//    glutInitWindowSize(W,H);
//    glutCreateWindow("VTK-m Rendering");
//    glutDisplayFunc(displayCall);
//    glutMotionFunc(mouseMove);
//    glutMouseFunc(mouseCall);
//    glutReshapeFunc(reshape);

  vtkm::rendering::Color bg(0.2f, 0.2f, 0.2f, 1.0f);
  vtkm::rendering::CanvasRayTracer canvas;
  MapperRayTracer mapper(csg.GetCellSet());

  vtkm::rendering::Scene scene;
  scene.AddActor(vtkm::rendering::Actor(csg.GetCellSet(),
                                        csg.GetCoordinateSystem(),
                                        csg.GetField("radius"),
                                        vtkm::rendering::ColorTable("thermal")));

  //Create vtkm rendering stuff.
  view = new vtkm::rendering::View3D(scene, mapper, canvas, bg);
  view->Initialize();
  //glutMainLoop();
  view->Paint();
  view->SaveAs("reg3D.pnm");

//QImage img(canvas.GetWidth(), canvas.GetHeight(), QImage::Format_RGB32);
//typedef vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32, 4> > ColorBufferType;

//ColorBufferType::PortalConstControl colorPortal =
//  canvas.GetColorBuffer().GetPortalConstControl();
//for (vtkm::Id yIndex = canvas.GetHeight() - 1; yIndex >= 0; yIndex--)
//{
//  for (vtkm::Id xIndex = 0; xIndex < canvas.GetWidth(); xIndex++)
//  {
//    vtkm::Vec<vtkm::Float32, 4> tuple =
//      colorPortal.Get(yIndex*canvas.GetWidth() + xIndex);
//    QRgb value;
//    value = qRgb(tuple[0]*255, tuple[1]*255, tuple[2]*255);
//    img.setPixel(xIndex, (canvas.GetHeight() -1) - yIndex, value);
//  }
//}

//QImageWriter writer;
//writer.setFormat("png");
//writer.setFileName("reg3D.png");
//writer.write(img);
}
}

#endif // MC_mc_driver_Manager_VTKm_hh

//---------------------------------------------------------------------------//
//                 end of Manager_VTKm.hh
//---------------------------------------------------------------------------//
