#ifndef TREE_H
#define TREE_H


#include <vector>
#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderExplicit.h>


class Tree{
    friend class TreeBuilder;
public:
  typedef vtkm::Vec<vtkm::Float32, 3> vec3;

  Tree(std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dsb):
    dataSetBuilder(dsb),
  vtx_cnt(0),
  cell_cnt(0){
  }

  void vtxPush();
  void idxvtxPush();
  void AddCell(const vtkm::UInt8 shape);
  void AddPoint(vtkm::Float32, vtkm::Float32, vtkm::Float32);
  void AddCellPoint(vtkm::Id idx);
  void AddCellPoint();
  void Create();
  vtkm::UInt32 getCnt(vtkm::Id idx){return child_cnt[idx];}
  vtkm::UInt32 getIdx(vtkm::Id idx){return child_idx[idx];}
  vtkm::UInt32 getVtx(vtkm::Id idx){return child_vtx[idx];}

  void test(int idx);
  void recurseTest(int _cnt,
                   int idx,
                         const vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl pts,
                         const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal,
                         const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal,
                         const vtkm::Vec<vtkm::Float32,3> &lower,
                         const vtkm::Vec<vtkm::Float32,3> &upper
                         );

  vtkm::cont::DataSet &getCSG(){return csg;}
protected:

  std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx, child_idx_vtx;
  vtkm::UInt32 vtx_cnt, cell_cnt;
  std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dataSetBuilder;
  vtkm::cont::DataSet csg;
};



#endif // TREE_H

