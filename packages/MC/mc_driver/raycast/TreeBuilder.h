#ifndef TREEBUILDER_H
#define TREEBUILDER_H

#include "Tree.h"

class TreeBuilder
{
public:
    TreeBuilder(std::shared_ptr<Tree> _treePtr,
               const profugus::Core::Array_t &ot,
               std::vector<vtkm::Float32> &radii,
               const def::Space_Vector &corner):
        treePtr(_treePtr)
    {
      treePtr->child_cnt.push_back(ot.size());
      treePtr->child_idx.push_back(1);
      treePtr->AddCell(vtkm::CELL_SHAPE_TETRA);
      treePtr->AddPoint(corner[0], corner[1], corner[2]);
      treePtr->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
      treePtr->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
      treePtr->AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());

      treePtr->vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
      treePtr->idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
      treePtr->AddCellPoint();
      treePtr->AddCellPoint();
      treePtr->AddCellPoint();
      treePtr->AddCellPoint();
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

      for (int i=2; i<treePtr->child_cnt.size(); i++){
          treePtr->child_cnt[i-1] = treePtr->child_cnt[i];
      }
      for (int i=2; i<treePtr->child_vtx.size(); i++){
          treePtr->child_vtx[i-1] = treePtr->child_vtx[i];
      }
      treePtr->Create();
    }

    std::shared_ptr<Tree> getTree()
    {
        return treePtr;
    }


    std::shared_ptr<Tree> treePtr;
protected:
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
             )
    {
      treePtr->child_cnt.push_back( 1 );
      new_offset++;
      treePtr->child_idx.push_back(new_offset);

    }


    template<class T>
    void cell_set_dispatch(
                           const profugus::RTK_Array<T> &ot,
                           std::vector<vtkm::Float32> &radii,
                           const def::Space_Vector &corner,
                           int level_cnt,
                           int &new_offset);
    void cell_set_dispatch(
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

          treePtr->AddPoint(new_low[def::X],
                  new_low[def::Y],
                  new_low[def::Z]);//dataSetBuilder->AddPoint(new_low[def::X],new_low[def::Y], new_low[def::Z]);
          treePtr->AddPoint(new_up[def::X],
                  new_up[def::Y],
                  new_up[def::Z]);//dataSetBuilder->AddPoint(new_up[def::X],new_up[def::Y],new_up[def::Z]);
          treePtr->AddPoint(0,0,0);//dataSetBuilder->AddPoint(0,0,0);
          treePtr->AddCell(vtkm::CELL_SHAPE_TRIANGLE);//dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TRIANGLE);
          treePtr->vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
          treePtr->idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
          treePtr->AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
          treePtr->AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
          treePtr->AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
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
        treePtr->AddPoint(lower[0], lower[1],lower[2]);//dataSetBuilder->AddPoint(lower[0], lower[1], lower[2]);
        treePtr->AddPoint(upper[0], upper[1],upper[2]);//dataSetBuilder->AddPoint(upper[0], upper[1], upper[2]);

        treePtr->AddCell(vtkm::CELL_SHAPE_LINE);//dataSetBuilder->AddCell(vtkm::CELL_SHAPE_LINE);
        treePtr->vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
        treePtr->idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
        treePtr->AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
        radii.push_back(0.);
        treePtr->AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
        radii.push_back(0.);

        //box

    //      quick_stop = 1;
      }
    }



};

template<class T>
void TreeBuilder::add(
               const profugus::RTK_Array<T> &ot,
               std::vector<vtkm::Float32> &radii,
         const def::Space_Vector &cur_corner,
         vtkm::Int32 level,
         vtkm::Int32 &new_offset
         ){
  treePtr->child_cnt.push_back( ot.size());
  new_offset += ot.size();
  treePtr->child_idx.push_back(new_offset);

  treePtr->vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
  treePtr->idxvtxPush();//.push_back(treePtr->cell_cnt);
  treePtr->AddCell(vtkm::CELL_SHAPE_TETRA);
  treePtr->AddPoint(cur_corner[0], cur_corner[1], cur_corner[2]);
  treePtr->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  treePtr->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
  treePtr->AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());

//  dataSetBuilder->AddCellPoint(cell_cnt++);
//  dataSetBuilder->AddCellPoint(cell_cnt++);
//  dataSetBuilder->AddCellPoint(cell_cnt++);
//  dataSetBuilder->AddCellPoint(cell_cnt++);
  treePtr->AddCellPoint();
  treePtr->AddCellPoint();
  treePtr->AddCellPoint();
  treePtr->AddCellPoint();
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
  radii.push_back(0.);
}


template<class T>
void TreeBuilder::cell_set_dispatch(
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

#endif // TREEBUILDER_H

