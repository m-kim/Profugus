#include "Tree.h"




void Tree::recurseTest(int _cnt,
                       int _idx,
                       const vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,3>>::PortalControl pts,
                       const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal,
                       const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal,
                        const vtkm::Vec<vtkm::Float32,3> &lower,
                        const vtkm::Vec<vtkm::Float32,3> &upper
                       )
{
    _idx = child_idx[_idx];

    for (int i=0; i<_cnt; i++){
        int _vtx = child_vtx[_idx + i];
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
            recurseTest(child_cnt[_idx+i], _idx + i, pts, ShapesPortal,OffsetsPortal, lower, upper);
        }
    }
}

void Tree::test(int _idx)
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
    int _vtx = child_vtx[_idx];
    vtkm::UInt8 type = ShapesPortal.Get(_vtx);
    vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);
    lower = pts.Get(cur_offset);
    upper = pts.Get(cur_offset + 1);

    recurseTest(child_cnt[_idx], _idx, pts, ShapesPortal, OffsetsPortal, lower, upper);
}


void Tree::vtxPush()
{
    child_vtx.push_back(vtx_cnt++);
}

void Tree::idxvtxPush()
{
    child_idx_vtx.push_back(cell_cnt);
}

void Tree::AddCell(const vtkm::UInt8 shape)
{
    dataSetBuilder->AddCell(shape);
}

void Tree::AddPoint(vtkm::Float32 x, vtkm::Float32 y, vtkm::Float32 z)
{
    dataSetBuilder->AddPoint(x,y,z);
}

void Tree::AddCellPoint(vtkm::Id idx)
{
    dataSetBuilder->AddCellPoint(idx);
}

void Tree::AddCellPoint()
{
    AddCellPoint(cell_cnt++);
}

void Tree::Create()
{
    csg = dataSetBuilder->Create();
}
