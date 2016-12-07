//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   MC/mc_driver/mc.cc
 * \author Thomas M. Evans
 * \date   Wed Mar 12 22:24:55 2014
 * \brief  SPn Mini-App executable.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include <string>
#include <iostream>
#include <algorithm>

#include "harness/DBC.hh"
#include "comm/Timer.hh"
#include "comm/global.hh"
#include "comm/Timing_Diagnostics.hh"
#include "comm/P_Stream.hh"
#include "utils/Definitions.hh"
#include "Manager_Base.hh"
#include "Manager_Builder.hh"

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

// Parallel specs.
int node  = 0;
int nodes = 0;

//---------------------------------------------------------------------------//
// Print instructions on how to run the neutronics executable

void print_usage()
{
    if (node == 0)
    {
        std::cout << "Usage: xmc -i XMLFILE" << std::endl;
        std::cout << "Executes the xmc executable using XMLFILE "
                  << "as the input file." << std::endl;
        exit(1);
    }
}

//---------------------------------------------------------------------------//
// Parse the input arguments

std::string parse_input_arguments(const def::Vec_String &arguments)
{
    // First, search for "-h" or "--help"
    if (std::find(arguments.begin(), arguments.end(), "-h") != arguments.end()
        || std::find(arguments.begin(), arguments.end(), "--help") !=
       arguments.end())
    {
        print_usage();
    }

    // Search for "-i"
    auto iter = std::find(arguments.begin(), arguments.end(), "-i");
    if (iter == arguments.end() || iter == arguments.end()-1)
    {
        if (node == 0)
        {
            std::cout << std::endl << "ERROR: Missing xml input filename."
                      << std::endl << std::endl;
        }
        print_usage();
    }

    // Get the xml filename
    const std::string &xml_filename = *(iter+1);
    if (xml_filename.empty())
    {
        if (node == 0)
        {
            std::cout << std::endl << "ERROR: Missing xml input filename."
                      << std::endl << std::endl;
        }
        print_usage();
    }

    return xml_filename;
}

int lastx=-1, lasty=-1;
vtkm::rendering::View3D *view = nullptr;

void tmp()
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

//---------------------------------------------------------------------------//

int main(int argc, char *argv[])
{
    profugus::initialize(argc, argv);

    // start timing
    profugus::global_barrier();
    profugus::Timer timer;
    timer.start();

    // nodes
    node  = profugus::node();
    nodes = profugus::nodes();

    profugus::pcout << "=======================================\n"
                    << "    Profugus MC Mini-APP               \n"
                    << "    (C) ORNL, Battelle, 2014           \n"
                    << "=======================================\n"
                    << profugus::endl;

    // process input arguments
    def::Vec_String arguments(argc - 1);
    std::string     xml_file;
    for (int c = 1; c < argc; c++)
    {
        arguments[c - 1] = argv[c];
    }
    xml_file = parse_input_arguments(arguments);

    try
    {
        // make the manager
        auto manager = mc::Manager_Builder::build(xml_file);

        // solve the problem
        manager->solve();

        // output
        manager->output();
    }
    catch (const profugus::assertion &a)
    {
        std::cout << "Caught profugus assertion " << a.what() << std::endl;
        exit(1);
    }
    catch (const std::exception &a)
    {
        std::cout << "Caught standard assertion " << a.what() << std::endl;
        exit(1);
    }
    catch (...)
    {
        std::cout << "Caught assertion of unknown origin." << std::endl;
        exit(1);
    }

    // process and output timing diagnostics
    profugus::global_barrier();
    timer.stop();
    double total = timer.TIMER_CLOCK();
    profugus::Timing_Diagnostics::report(std::cout, total);

    // output final timing
    profugus::pcout << "\n" << "Total execution time : "
                    << profugus::scientific
                    << profugus::setprecision(4)
                    << total << " seconds." << profugus::endl;

    profugus::finalize();

    tmp();
    return 0;
}

//---------------------------------------------------------------------------//
//                 end of mc.cc
//---------------------------------------------------------------------------//
