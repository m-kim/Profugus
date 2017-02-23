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
           ){}

  template<class T>
  void cell_set_dispatch(
                         const profugus::RTK_Array<T> &ot,
                         std::vector<vtkm::Float32> &radii,
                         const def::Space_Vector &corner,
                         int level_cnt,
                         int offset);
public:
  std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx, child_idx_vtx;
  vtkm::UInt32 vtx_cnt, cell_cnt;
  std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dataSetBuilder;
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
  dataSetBuilder->AddCell(vtkm::CELL_SHAPE_HEXAHEDRON);
  dataSetBuilder->AddPoint(cur_corner[0], cur_corner[1], cur_corner[2]);
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  dataSetBuilder->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());

  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
}


#endif // TREE_H

