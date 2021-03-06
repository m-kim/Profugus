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
#include "utils/Definitions.hh"
#include "Manager.hh"

#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/testing/MakeTestDataSet.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/io/writer/VTKDataSetWriter.h>

#include "raycast/MapperRayTracer.h"
#include "raycast/Tree.h"
#include "raycast/TreeBuilder.h"

#include <vtkm/rendering/View3D.h>
#include <vtkm/rendering/ColorTable.h>
#include <iostream>
#include <fstream>
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
    Manager_VTKm(){}

    // Output.
    void output(){  Manager<Geometry>::output();}
private:
    void raycast(){}

};



template<>
class Manager_VTKm<profugus::Core> : public Manager<profugus::Core>
{
public:
  Manager_VTKm()
      : Manager<profugus::Core>()
  {
    lastx = lasty = -1;
    view = nullptr;
  }
  void output()
  {
    Manager<profugus::Core>::output();

    std::cout << "WOOT" << std::endl;
    raycast();
  }




  def::Space_Vector total_length(const profugus::Core::Array_t ot)
  {
    def::Space_Vector lower, upper, tot_lower, tot_upper;
    for (int k = 0; k < ot.size(def::K); ++k)
    {
      ot.get_extents(lower, upper);
      tot_lower[def::Z] += lower[def::Z];
      tot_upper[def::Z] += upper[def::Z];

    }
    for (int j = 0; j < ot.size(def::J); ++j)
    {
      ot.get_extents(lower, upper);
      tot_lower[def::Y] += lower[def::Y];
      tot_upper[def::Y] += upper[def::Y];
    }
    for (int i = 0; i < ot.size(def::I); ++i)
    {

      //int n = ot.id(i, j, k);
      ot.get_extents(lower, upper);
      tot_lower[def::X] += lower[def::X];
      tot_upper[def::X] += upper[def::X];

    }

    return upper - lower;

  }


  void raycast()
  {
      typedef VTKM_DEFAULT_DEVICE_ADAPTER_TAG DeviceAdapter;
    std::shared_ptr<profugus::Core > sp_geo_core = this->d_geometry;
    profugus::Core::Array_t ot = sp_geo_core->array();
    def::Space_Vector corner = ot.corner();
#if 1

    quick_stop = 0;
    std::shared_ptr<Tree<DeviceAdapter>> treePtr(new Tree<DeviceAdapter>());
    std::shared_ptr<TreeBuilder<DeviceAdapter>> tb(new TreeBuilder<DeviceAdapter>(treePtr));
    //tb->build(ot, corner, radii);

    //tb->write();
    tb->load("dataset.vtk");
    dataSetFieldAdd.AddPointField(tb->getCSG(), "radius", radii);

#else


    //cylinder
    dataSetBuilder->AddPoint(1, 0.25, 1);
    dataSetBuilder->AddPoint(1, .75, 1);

    //box
    dataSetBuilder->AddPoint(0.25, 0.25, 0.25);
//    dataSetBuilder->AddPoint(0.75, .75, 0.75);

//    //sphere
//    dataSetBuilder->AddPoint(1.5, 0.5, 1);

    dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TRIANGLE);
    dataSetBuilder->AddCellPoint(0);
    dataSetBuilder->AddCellPoint(1);
    dataSetBuilder->AddCellPoint(2);
    radii.push_back(0.2);
    radii.push_back(0.2);


//    dataSetBuilder->AddCell(vtkm::CELL_SHAPE_VERTEX);
//    dataSetBuilder->AddCellPoint(4);
//    radii.push_back(0.1);
    vtkm::cont::DataSet csg = dataSetBuilder->Create();


//    //cylinder
//    radii.push_back(0.2/63.);
//    radii.push_back(0.2/63.);
//    //box
//    radii.push_back(0);
//    radii.push_back(0);
//    //sphere
//    radii.push_back(0.1);

    dataSetFieldAdd.AddPointField(csg, "radius", radii);
#endif
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
    vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray = tb->getCnt();
    vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray = tb->getIdx();
    vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray = tb->getVtx();
    MapperRayTracer mapper(tb->getCSG().GetCellSet(), cntArray, idxArray, vtxArray );

    vtkm::rendering::Scene scene;
    scene.AddActor(vtkm::rendering::Actor(tb->getCSG().GetCellSet(),
                                          tb->getCSG().GetCoordinateSystem(),
                                          tb->getCSG().GetField("radius"),
                                          vtkm::rendering::ColorTable("cool2warm")));

    //Create vtkm rendering stuff.
    vtkm::rendering::Camera new_cam;
    vtkm::Bounds spatialBounds = scene.GetSpatialBounds();
    new_cam.SetPosition(vtkm::Vec<vtkm::Float32, 3>(0,1,0));
    new_cam.SetViewUp(vtkm::Vec<vtkm::Float32,3>(0,0,1));
    new_cam.ResetToBounds(spatialBounds);
    vtkm::Vec<vtkm::Float32, 3> pos = new_cam.GetPosition();
    pos[1] *= 1;
    pos[2] += pos[2] * 4.0;
    new_cam.SetPosition(pos);

#if 0
    treePtr->test(0);
#else
    vtkm::Vec<vtkm::Float32,3> wtf(2,2,2);
    //test_flat(wtf);
    view = new vtkm::rendering::View3D(scene, mapper, canvas, new_cam, bg);
    new_cam.SetPosition(pos);
    view->SetCamera(new_cam);

    view->Initialize();
    //glutMainLoop();
    view->Paint();
    view->SaveAs("reg3D.pnm");
#endif
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
private:
  bool quick_stop;
  vtkm::cont::DataSetFieldAdd dataSetFieldAdd;
  std::vector<vtkm::Float32> radii;
  def::Space_Vector tot_len;
  int lastx, lasty;
  vtkm::rendering::View3D *view;

};

}

#endif // MC_mc_driver_Manager_VTKm_hh

//---------------------------------------------------------------------------//
//                 end of Manager_VTKm.hh
//---------------------------------------------------------------------------//
