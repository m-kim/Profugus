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


  void cell_set_dispatch(const profugus::RTK_Cell &ot,
                         const def::Space_Vector &corner,
                         int level,
                         int offset)
  {
#ifdef DEBUG_VTKM
    std::cout << corner[0] << " " << corner[1] << " " << corner[2] << " ";
    std::cout << ot.pitch(0) << " " << ot.pitch(1) << " ";
    std::cout << std::endl;
#endif
    if (ot.radii().size() > 0){
//      if (!quick_stop){
        def::Space_Vector lower, upper;
        ot.get_extents(lower, upper);
        def::Space_Vector new_low, new_up;
        new_low[def::X] = new_up[def::X] = (corner[def::X] - lower[def::X]);
        new_low[def::Y] = new_up[def::Y] = (corner[def::Y] - lower[def::Y]);

        new_low[def::Z] = (corner[def::Z]);
        new_up[def::Z] = (corner[def::Z] + ot.height());

        dataSetBuilder.AddPoint(new_low[def::X],
            new_low[def::Y],
            new_low[def::Z]);
        dataSetBuilder.AddPoint(new_up[def::X],
            new_up[def::Y],
            new_up[def::Z]);
        dataSetBuilder.AddPoint(0,0,0);
        dataSetBuilder.AddCell(vtkm::CELL_SHAPE_TRIANGLE);
        child_vtx.push_back(cell_cnt);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        radii.push_back(ot.radii()[0]);
        radii.push_back(ot.radii()[0]);
        radii.push_back(ot.radii()[0]);
//        quick_stop = 1;
//      }
    }
    else {
      //box
      def::Space_Vector lower, upper;
      ot.get_extents(lower, upper);
      upper = corner + upper - lower;
      lower = corner;
      dataSetBuilder.AddPoint(lower[0], lower[1], lower[2]);
      dataSetBuilder.AddPoint(upper[0], upper[1], upper[2]);

      dataSetBuilder.AddCell(vtkm::CELL_SHAPE_LINE);
      child_vtx.push_back(cell_cnt);
      dataSetBuilder.AddCellPoint(cell_cnt++);
      radii.push_back(0.);
      dataSetBuilder.AddCellPoint(cell_cnt++);
      radii.push_back(0.);

      //box

//      quick_stop = 1;
    }
  }

  template<class T>
  void add(const profugus::RTK_Array<T> &ot,
           const def::Space_Vector &cur_corner,
           vtkm::Int32 level,
           vtkm::Int32 &new_offset
           ){
    child_cnt.push_back( ot.size());
    new_offset += ot.size();
    child_idx.push_back(new_offset);

    child_vtx.push_back(cell_cnt);
    dataSetBuilder.AddCell(vtkm::CELL_SHAPE_HEXAHEDRON);
    dataSetBuilder.AddPoint(cur_corner[0], cur_corner[1], cur_corner[2]);
    dataSetBuilder.AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
    dataSetBuilder.AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
    dataSetBuilder.AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
    dataSetBuilder.AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
    dataSetBuilder.AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());

    dataSetBuilder.AddCellPoint(cell_cnt++);
    dataSetBuilder.AddCellPoint(cell_cnt++);
    dataSetBuilder.AddCellPoint(cell_cnt++);
    dataSetBuilder.AddCellPoint(cell_cnt++);
    dataSetBuilder.AddCellPoint(cell_cnt++);
    dataSetBuilder.AddCellPoint(cell_cnt++);
    radii.push_back(0.);
    radii.push_back(0.);
    radii.push_back(0.);
    radii.push_back(0.);
    radii.push_back(0.);
    radii.push_back(0.);
  }

  void add(const profugus::RTK_Cell &ot,
           const def::Space_Vector &cur_corner,
           vtkm::Int32 level,
           vtkm::Int32 &new_offset
           ){
  }

  template<class T>
  void cell_set_dispatch(const profugus::RTK_Array<T> &ot,
                         const def::Space_Vector &corner,
                         int level_cnt,
                         int offset)
{

      //wish I could just copy d_Nc_offset
  int new_offset = offset;
  def::Space_Vector cur_corner = corner;

  for (int k = 0; k < ot.size(def::K); ++k)
  {
    cur_corner[def::J] = corner[def::J];
    for (int j = 0; j < ot.size(def::J); ++j)
    {
      cur_corner[def::I] = corner[def::I];
      for (int i = 0; i < ot.size(def::I); ++i)
      {
          int n = ot.id(i, j, k);
          add(ot.object(n), cur_corner, level_cnt+1, new_offset);

          cur_corner[def::I] += ot.object(n).pitch(def::I);
        }
        int n =  ot.id(0,j,k);
        cur_corner[def::J] += ot.object(n).pitch(def::J);
    }
  }
  cur_corner = corner;
    for (int k = 0; k < ot.size(def::K); ++k)
    {
      cur_corner[def::J] = corner[def::J];
      for (int j = 0; j < ot.size(def::J); ++j)
      {
        cur_corner[def::I] = corner[def::I];
        for (int i = 0; i < ot.size(def::I); ++i)
        {

          int n = ot.id(i, j, k);
          cell_set_dispatch(ot.object(n),
                            cur_corner,
                            level_cnt+1,
                            new_offset);
          cur_corner[def::I] += ot.object(n).pitch(def::I);
        }
        int n =  ot.id(0,j,k);
        cur_corner[def::J] += ot.object(n).pitch(def::J);
      }

    }
    cur_corner[def::K] = corner[def::K];
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

  void test_flat(
          vtkm::Vec<vtkm::Float32,3> &pt)
  {
    vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal;
    vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal;
    vtkm::cont::DynamicCellSet dcs = csg.GetCellSet();

    vtkm::cont::CellSetExplicit<> cellSetExplicit = dcs.Cast<vtkm::cont::CellSetExplicit<> >();
    vtkm::Vec< vtkm::Id, 3> forceBuildIndices;
    cellSetExplicit.GetIndices(0, forceBuildIndices);
    ShapesPortal = cellSetExplicit.GetShapesArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell()).GetPortalConstControl();
//    const vtkm::cont::Field *scalarField;

    vtkm::cont::DynamicArrayHandleCoordinateSystem coords = csg.GetCoordinateSystem().GetData();
    vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>> points;
    coords.CopyTo(points);

   vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl wtf = points.GetPortalControl();
   int stack[64], s_cnt[64], s_i[64], stack_ptr;
   int _idx = 0;//
   int _cnt = 1;//child_cnt[_idx];
   //_idx= child_idx[_idx];
    int _vtx = child_vtx[_idx];
    vtkm::Vec<vtkm::Float32,3> lower, upper;
    //vtkm::Id cur_offset = coords.GetNumberOfValues();

    stack_ptr = 0;
    stack[stack_ptr] = _idx;
    s_cnt[stack_ptr] = _cnt;
    stack_ptr++;
    for (int i=0; i<_cnt;){
      _vtx = child_vtx[_idx + i];
      lower = wtf.Get(_vtx);
      upper = wtf.Get( _vtx + 1);

      if (upper[0] > pt[0] &&
              upper[1] > pt[1] &&
              upper[2] > pt[2] &&
              pt[0] > lower[0] &&
              pt[1] > lower[1] &&
              pt[2] > lower[2]){
          if(_idx != 0 &&_cnt == 1){
              //reached a leaf
            stack_ptr--;
            _idx = stack[stack_ptr];
            _cnt = s_cnt[stack_ptr];
            i = s_i[stack_ptr];
            i++;
          }
          else{
            //push onto stack
            stack[stack_ptr] = _idx;
            s_cnt[stack_ptr] = _cnt;
            s_i[stack_ptr] = i;


            _cnt = child_cnt[_idx];
            _idx = child_idx[_idx];
            _vtx = child_vtx[_idx];
            i = 0;
            stack_ptr++;
          }
      }
      else{
          //doesn't matter, do nothing
        i++;

      }
    }
  }

  void raycast()
  {
    std::shared_ptr<profugus::Core > sp_geo_core = this->d_geometry;
    profugus::Core::Array_t ot = sp_geo_core->array();
    def::Space_Vector corner = ot.corner();
#if 1
    cell_cnt = 0;
quick_stop = 0;
        child_cnt.push_back(ot.size());
        child_idx.push_back(1);
        dataSetBuilder.AddCell(vtkm::CELL_SHAPE_HEXAHEDRON);
        dataSetBuilder.AddPoint(corner[0], corner[1], corner[2]);
        dataSetBuilder.AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        dataSetBuilder.AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        dataSetBuilder.AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        dataSetBuilder.AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        dataSetBuilder.AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());

        child_vtx.push_back(cell_cnt);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        dataSetBuilder.AddCellPoint(cell_cnt++);
        radii.push_back(0.);
        radii.push_back(0.);
        radii.push_back(0.);
        radii.push_back(0.);
        radii.push_back(0.);
        radii.push_back(0.);

    tot_len = total_length(ot);
    cell_set_dispatch(ot,
                      corner,                      
                      0,
                      ot.size());
    csg = dataSetBuilder.Create();
    dataSetFieldAdd.AddPointField(csg, "radius", radii);
#else


    //cylinder
    dataSetBuilder.AddPoint(1, 0.25, 1);
    dataSetBuilder.AddPoint(1, .75, 1);

    //box
    dataSetBuilder.AddPoint(0.25, 0.25, 0.25);
//    dataSetBuilder.AddPoint(0.75, .75, 0.75);

//    //sphere
//    dataSetBuilder.AddPoint(1.5, 0.5, 1);

    dataSetBuilder.AddCell(vtkm::CELL_SHAPE_TRIANGLE);
    dataSetBuilder.AddCellPoint(0);
    dataSetBuilder.AddCellPoint(1);
    dataSetBuilder.AddCellPoint(2);
    radii.push_back(0.2);
    radii.push_back(0.2);


//    dataSetBuilder.AddCell(vtkm::CELL_SHAPE_VERTEX);
//    dataSetBuilder.AddCellPoint(4);
//    radii.push_back(0.1);
    vtkm::cont::DataSet csg = dataSetBuilder.Create();


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
    MapperRayTracer mapper(csg.GetCellSet());

    vtkm::rendering::Scene scene;
    scene.AddActor(vtkm::rendering::Actor(csg.GetCellSet(),
                                          csg.GetCoordinateSystem(),
                                          csg.GetField("radius"),
                                          vtkm::rendering::ColorTable("thermal")));

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

    vtkm::Vec<vtkm::Float32,3> wtf(2,2,2);
    test_flat(wtf);
    view = new vtkm::rendering::View3D(scene, mapper, canvas, new_cam, bg);
    new_cam.SetPosition(pos);
    view->SetCamera(new_cam);

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
private:
  bool quick_stop;
  vtkm::cont::DataSetBuilderExplicitIterative dataSetBuilder;
  vtkm::cont::DataSetFieldAdd dataSetFieldAdd;
  std::vector<vtkm::Float32> radii;
  std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx;
  def::Space_Vector tot_len;
  int lastx, lasty, cell_cnt;
  vtkm::rendering::View3D *view;
  vtkm::cont::DataSet csg;
};

}

#endif // MC_mc_driver_Manager_VTKm_hh

//---------------------------------------------------------------------------//
//                 end of Manager_VTKm.hh
//---------------------------------------------------------------------------//
