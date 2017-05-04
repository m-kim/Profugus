#ifndef TREE_H
#define TREE_H


#include <vector>
#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderExplicit.h>

template<typename DeviceAdapterTag>
class Tree{
private:
  typedef typename vtkm::cont::ArrayHandle<vtkm::UInt32>
      ::template ExecutionTypes<DeviceAdapterTag>::Portal PortalType;

public:
  typedef vtkm::Vec<vtkm::Float32, 3> vec3;

  Tree(std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dsb):
    dataSetBuilder(dsb)
  {
  }

  void vtxPush();
  void idxvtxPush();
  void AddCell(const vtkm::UInt8 shape);
  void AddPoint(vtkm::Float32, vtkm::Float32, vtkm::Float32);
  void AddCellPoint(vtkm::Id idx);
  void AddCellPoint();
  void Create();
  vtkm::UInt32 getCnt(vtkm::Id id){return cnt.Get(id);}
  vtkm::UInt32 getIdx(vtkm::Id id){return idx.Get(id);}
  vtkm::UInt32 getVtx(vtkm::Id id){return vtx.Get(id);}

  void SetCnt(vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray){
    cnt = cntArray.PrepareForInPlace(DeviceAdapterTag());

  }
  void SetVtx(vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray){
      vtx = vtxArray.PrepareForInPlace(DeviceAdapterTag());

  }
  void SetIdx(vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray){
      idx = idxArray.PrepareForInPlace(DeviceAdapterTag());

  }

  void recurseTest(int _cnt,
                         int _idx,
                         const vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl pts,
                         const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal,
                         const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal,
                          const vtkm::Vec<vtkm::Float32,3> &lower,
                          const vtkm::Vec<vtkm::Float32,3> &upper
                         )
  {
      _idx = idx.Get(_idx);

      for (int i=0; i<_cnt; i++){
          int _vtx = vtx.Get(_idx + i);
          vtkm::Vec<vtkm::Float32, 3> box_ll, box_ur;
          vtkm::UInt8 type = ShapesPortal.Get(_vtx);
          vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);

          if(type == vtkm::CELL_SHAPE_TETRA){
              box_ll = pts.Get(cur_offset);
              box_ur = pts.Get(cur_offset + 1);

              if (lower[0] > box_ll[0] ||
                      lower[1] > box_ll[1] ||
                      lower[2] > box_ll[2] ||
                      upper[0] < box_ur[0] ||
                      upper[1] < box_ur[1] ||
                      upper[2] < box_ur[2]){
                  std::cout << "nope: " << box_ll << " " << box_ur << std::endl;
              }
              recurseTest(cnt.Get(_idx+i), _idx + i, pts, ShapesPortal,OffsetsPortal, lower, upper);
          }
      }
  }

  void test(int _idx)
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

     vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl pts = points.GetPortalControl();
     OffsetsPortal = cellSetExplicit.GetIndexOffsetArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell()).GetPortalConstControl();

      vtkm::Vec<vtkm::Float32, 3> lower, upper;
      int _vtx = vtx.Get(_idx);
      vtkm::UInt8 type = ShapesPortal.Get(_vtx);
      vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);
      lower = pts.Get(cur_offset);
      upper = pts.Get(cur_offset + 1);

      recurseTest(cnt.Get(_idx), _idx, pts, ShapesPortal, OffsetsPortal, lower, upper);
  }

  vtkm::cont::DataSet &getCSG(){return csg;}
  std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dataSetBuilder;
  vtkm::cont::DataSet csg;
protected:

  PortalType cnt, vtx, idx;
};



#endif // TREE_H

