#include "Tree.h"


void Tree::cell_set_dispatch(
                             const profugus::RTK_Cell &ot,
                             std::vector<vtkm::Float32> &radii,
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

      dataSetBuilder->AddPoint(new_low[def::X],
          new_low[def::Y],
          new_low[def::Z]);
      dataSetBuilder->AddPoint(new_up[def::X],
          new_up[def::Y],
          new_up[def::Z]);
      dataSetBuilder->AddPoint(0,0,0);
      dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TRIANGLE);
      child_vtx.push_back(vtx_cnt++);
      child_idx_vtx.push_back(cell_cnt);
      dataSetBuilder->AddCellPoint(cell_cnt++);
      dataSetBuilder->AddCellPoint(cell_cnt++);
      dataSetBuilder->AddCellPoint(cell_cnt++);
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
    dataSetBuilder->AddPoint(lower[0], lower[1], lower[2]);
    dataSetBuilder->AddPoint(upper[0], upper[1], upper[2]);

    dataSetBuilder->AddCell(vtkm::CELL_SHAPE_LINE);
    child_vtx.push_back(vtx_cnt++);
    child_idx_vtx.push_back(cell_cnt);
    dataSetBuilder->AddCellPoint(cell_cnt++);
    radii.push_back(0.);
    dataSetBuilder->AddCellPoint(cell_cnt++);
    radii.push_back(0.);

    //box

//      quick_stop = 1;
  }
}
void Tree::build(
                 const profugus::Core::Array_t &ot,
                 std::vector<vtkm::Float32> &radii,
                 const def::Space_Vector &corner)
{
  child_cnt.push_back(ot.size());
  child_idx.push_back(1);
  dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TETRA);
   dataSetBuilder->AddPoint(corner[0], corner[1], corner[2]);
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
\
  child_vtx.push_back(vtx_cnt++);
  child_idx_vtx.push_back(cell_cnt);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  dataSetBuilder->AddCellPoint(cell_cnt++);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);

  int offset = ot.size();
  cell_set_dispatch(
                    ot,
                    radii,
                    corner,
                    0,
                    offset);

  for (int i=2; i<child_cnt.size(); i++){
      child_cnt[i-1] = child_cnt[i];
  }
  for (int i=2; i<child_vtx.size(); i++){
      child_vtx[i-1] = child_vtx[i];
  }
  csg = dataSetBuilder->Create();
}

void Tree::add(
         const profugus::RTK_Cell &ot,
         std::vector<vtkm::Float32> &radii,
         const def::Space_Vector &cur_corner,
         vtkm::Int32 level,
         vtkm::Int32 &new_offset
         )
{
  child_cnt.push_back( 1 );
  new_offset++;
  child_idx.push_back(new_offset);

}

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


