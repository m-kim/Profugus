#ifndef TREEBUILDER_H
#define TREEBUILDER_H

#include "Tree.h"

template<typename DeviceAdapterTag>
class TreeBuilder
{
public:
    TreeBuilder(std::shared_ptr<Tree<DeviceAdapterTag>> _treePtr ):
        vtx_cnt(0),
        cell_cnt(0),
        treePtr(_treePtr)
    {

    }

    void build(const profugus::Core::Array_t &ot, const def::Space_Vector &corner, std::vector<vtkm::Float32> &radii)
    {
        child_cnt.push_back(ot.size());
        child_idx.push_back(1);
        AddCell(vtkm::CELL_SHAPE_TETRA);
        AddPoint(corner[0], corner[1], corner[2]);
        AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());
        AddPoint(corner[0] + ot.pitch(def::I), corner[1] + ot.pitch(def::J), corner[2] + ot.height());

        vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
        idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
        AddCellPoint();
        AddCellPoint();
        AddCellPoint();
        AddCellPoint();
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

        vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray = vtkm::cont::make_ArrayHandle(&child_cnt[0], child_cnt.size());
        vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray = vtkm::cont::make_ArrayHandle(&child_vtx[0], child_vtx.size());
        vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray = vtkm::cont::make_ArrayHandle(&child_idx[0], child_idx.size());


        treePtr->SetCnt(cntArray);
        treePtr->SetVtx(vtxArray);
        treePtr->SetIdx(idxArray);
        Create();
    }

    std::shared_ptr<Tree<DeviceAdapterTag>> getTree()
    {
        return treePtr;
    }
    vtkm::UInt32 getCnt(vtkm::Id idx){return child_cnt[idx];}
    vtkm::UInt32 getIdx(vtkm::Id idx){return child_idx[idx];}
    vtkm::UInt32 getVtx(vtkm::Id idx){return child_vtx[idx];}


    std::shared_ptr<Tree<DeviceAdapterTag>> treePtr;
protected:
    template<class T>
    void add(
             const profugus::RTK_Array<T> &ot,
             std::vector<vtkm::Float32> &radii,
             const def::Space_Vector &cur_corner,
             vtkm::Int32 level,
             vtkm::Int32 &new_offset
             )
    {
      child_cnt.push_back( ot.size());
      new_offset += ot.size();
      child_idx.push_back(new_offset);

      vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
      idxvtxPush();//.push_back(treePtr->cell_cnt);
      AddCell(vtkm::CELL_SHAPE_TETRA);
      AddPoint(cur_corner[0], cur_corner[1], cur_corner[2]);
      AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
      AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());
      AddPoint(cur_corner[0] + ot.pitch(def::I), cur_corner[1] + ot.pitch(def::J), cur_corner[2] + ot.height());

    //  dataSetBuilder->AddCellPoint(cell_cnt++);
    //  dataSetBuilder->AddCellPoint(cell_cnt++);
    //  dataSetBuilder->AddCellPoint(cell_cnt++);
    //  dataSetBuilder->AddCellPoint(cell_cnt++);
      AddCellPoint();
      AddCellPoint();
      AddCellPoint();
      AddCellPoint();
      radii.push_back(0.);
      radii.push_back(0.);
      radii.push_back(0.);
      radii.push_back(0.);
    }



    void add(
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


    template<class T>
    void cell_set_dispatch(
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

          AddPoint(new_low[def::X],
                  new_low[def::Y],
                  new_low[def::Z]);//dataSetBuilder->AddPoint(new_low[def::X],new_low[def::Y], new_low[def::Z]);
          AddPoint(new_up[def::X],
                  new_up[def::Y],
                  new_up[def::Z]);//dataSetBuilder->AddPoint(new_up[def::X],new_up[def::Y],new_up[def::Z]);
          AddPoint(0,0,0);//dataSetBuilder->AddPoint(0,0,0);
          AddCell(vtkm::CELL_SHAPE_TRIANGLE);//dataSetBuilder->AddCell(vtkm::CELL_SHAPE_TRIANGLE);
          vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
          idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
          AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
          AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
          AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
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
        AddPoint(lower[0], lower[1],lower[2]);//dataSetBuilder->AddPoint(lower[0], lower[1], lower[2]);
        AddPoint(upper[0], upper[1],upper[2]);//dataSetBuilder->AddPoint(upper[0], upper[1], upper[2]);

        AddCell(vtkm::CELL_SHAPE_LINE);//dataSetBuilder->AddCell(vtkm::CELL_SHAPE_LINE);
        vtxPush();//child_vtx.push_back(treePtr->vtx_cnt++);
        idxvtxPush();//child_idx_vtx.push_back(treePtr->cell_cnt);
        AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
        radii.push_back(0.);
        AddCellPoint();//dataSetBuilder->AddCellPoint(treePtr->cell_cnt++);
        radii.push_back(0.);

        //box

    //      quick_stop = 1;
      }
    }
    void vtxPush()
    {
        child_vtx.push_back(vtx_cnt++);
    }

    void idxvtxPush()
    {
        child_idx_vtx.push_back(cell_cnt);
    }

    void AddCell(const vtkm::UInt8 shape)
    {
        treePtr->dataSetBuilder->AddCell(shape);
    }

    void AddPoint(vtkm::Float32 x, vtkm::Float32 y, vtkm::Float32 z)
    {
        treePtr->dataSetBuilder->AddPoint(x,y,z);
    }

    void AddCellPoint(vtkm::Id idx)
    {
        treePtr->dataSetBuilder->AddCellPoint(idx);
    }

    void AddCellPoint()
    {
        AddCellPoint(cell_cnt++);
    }

    void Create()
    {
        treePtr->csg = treePtr->dataSetBuilder->Create();
    }


    std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx, child_idx_vtx;
    vtkm::UInt32 vtx_cnt, cell_cnt;

};


#endif // TREEBUILDER_H

