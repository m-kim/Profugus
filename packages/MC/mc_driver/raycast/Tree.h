#ifndef TREE_H
#define TREE_H


#include <vector>
#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderExplicit.h>

#include <geometry/RTK_Geometry.hh>
#include "utils/Definitions.hh"

class Tree{
public:
  typedef vtkm::Vec<vtkm::Float32, 3> vec3;

  Tree(std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dsb):
    dataSetBuilder(dsb),
  vtx_cnt(0),
  cell_cnt(0){
  }

  void build(
             const profugus::Core::Array_t &ot,
             std::vector<vtkm::Float32> &radii,
             const def::Space_Vector &corner);

  void test(int idx);
  void recurseTest(int _cnt,
                   int idx,
                         const vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl pts,
                         const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal,
                         const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal,
                         const vtkm::Vec<vtkm::Float32,3> &lower,
                         const vtkm::Vec<vtkm::Float32,3> &upper
                         );

protected:
  void cell_set_dispatch(
                         const profugus::RTK_Cell &ot,
                         std::vector<vtkm::Float32> &radii,
                         const def::Space_Vector &corner,
                         int level,
                         int offset);

  template<class T>
  void add(
           const profugus::RTK_Array<T> &ot,
           std::vector<vtkm::Float32> &radii,
           const def::Space_Vector &cur_corner,
           vtkm::Int32 level,
           vtkm::Int32 &new_offset
           );


  void add(
           const profugus::RTK_Cell &ot,
           std::vector<vtkm::Float32> &radii,
           const def::Space_Vector &cur_corner,
           vtkm::Int32 level,
           vtkm::Int32 &new_offset
           );

  template<class T>
  void cell_set_dispatch(
                         const profugus::RTK_Array<T> &ot,
                         std::vector<vtkm::Float32> &radii,
                         const def::Space_Vector &corner,
                         int level_cnt,
                         int &new_offset);

public:
  std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx, child_idx_vtx;
  vtkm::UInt32 vtx_cnt, cell_cnt;
  std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dataSetBuilder;
  vtkm::cont::DataSet csg;
};

template<class T>
void Tree::add(
               const profugus::RTK_Array<T> &ot,
               std::vector<vtkm::Float32> &radii,
         const def::Space_Vector &cur_corner,
         vtkm::Int32 level,
         vtkm::Int32 &new_offset
         ){
  child_cnt.push_back( ot.size());
  new_offset += ot.size();
  child_idx.push_back(new_offset);

  child_vtx.push_back(vtx_cnt++);
  child_idx_vtx.push_back(cell_cnt);
  dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TETRA);
  dataSetBuilder->AddPoint(cur_corner[0], cur_corner[1], cur_corner[2]);
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());

  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
}
template<class T>
void Tree::cell_set_dispatch(
                             const profugus::RTK_Array<T> &ot,
                             std::vector<vtkm::Float32> &radii,
                       const def::Space_Vector &corner,
                       int level_cnt,
                       int &new_offset)
{

  //wish I could just copy d_Nc_offset
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
          add(ot.object(n), radii, cur_corner, level_cnt+1, new_offset);

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
        cell_set_dispatch(
                          ot.object(n),
                          radii,
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


#endif // TREE_H

