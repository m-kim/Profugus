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
template<class T>
void Tree::cell_set_dispatch(
                             const profugus::RTK_Array<T> &ot,
                             std::vector<vtkm::Float32> &radii,
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
void Tree::build(
                 const profugus::Core::Array_t &ot,
                 std::vector<vtkm::Float32> &radii,
                 const def::Space_Vector &corner)
{
  child_cnt.push_back(ot.size());
  child_idx.push_back(1);
  dataSetBuilder->AddCell(vtkm::CELL_SHAPE_HEXAHEDRON);
  dataSetBuilder->AddPoint(corner[0], corner[1], corner[2]);
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
  dataSetBuilder->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());

  child_vtx.push_back(vtx_cnt++);
  child_idx_vtx.push_back(cell_cnt);
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

  cell_set_dispatch(
                    ot,
                    radii,
                    corner,
                    0,
                    ot.size());
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
