#ifndef TREEINTERSECTOR_H
#define TREEINTERSECTOR_H

#include "Tree.h"

class TreeIntersector
{
public:

  typedef Tree::vec3 vec3;

  TreeIntersector(
      std::shared_ptr<Tree> _treePtr,
      const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl &sp,
      const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl &op)
    :treePtr(_treePtr),
      ShapesPortal(sp),
      OffsetsPortal(op)
  {

  }

  static
  vec3 cylinder(const vec3 &ray_start,
      const vec3 &ray_direction,
      const vec3 &p,
      const vec3 &q,
      float r);

  static
  vec3 box(const vec3 &ray_start,
           const vec3 &ray,
           const vec3 &lb,
           const vec3 &rt);

  static
  vec3 sphere(const vec3 &ray_start,
      const vec3 &ray_direction,
      const vec3 &center,
      float r);

  template<typename PointPortalType, typename ScalarPortalType>
  void query(
            const PointPortalType &points,
            const ScalarPortalType &scalars,
            const vtkm::Vec<vtkm::Float32, 3> &rayOrigin,
      const vtkm::Vec<vtkm::Float32, 3> &rayDir,
            vtkm::UInt8 &fin_type,
            vtkm::UInt8 &face,
            vtkm::Id &fin_offset,
            vec3 &fin_center,
            vtkm::Float32 &minDistance,
            vtkm::Id &hitIndex) const;


  std::shared_ptr<Tree> treePtr;

protected:
  const vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal;
  const vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal;
};

template<typename PointPortalType, typename ScalarPortalType>
void TreeIntersector::query(
          const PointPortalType &points,
          const ScalarPortalType &scalars,
          const vtkm::Vec<vtkm::Float32, 3> &rayOrigin,
    const vtkm::Vec<vtkm::Float32, 3> &rayDir,
          vtkm::UInt8 &fin_type,
          vtkm::UInt8 &face,
          vtkm::Id &fin_offset,
          vec3 &fin_center,
          vtkm::Float32 &minDistance,
          vtkm::Id &hitIndex) const
{

  vec3 cyl_top, cyl_bottom;
  vec3 center;
  vtkm::Float32 cyl_radius;
  vec3 box_ll, box_ur;

  int stack[64], s_cnt[64], s_i[64], stack_ptr;
  int _idx = 0;//
  int _cnt = 1;//child_cnt[_idx];
  //_idx= child_idx[_idx];
   int _vtx = treePtr->child_vtx[_idx];
   vtkm::Vec<vtkm::Float32,3> lower, upper;
   //vtkm::Id cur_offset = coords.GetNumberOfValues();
    vec3 ret;
   stack_ptr = 0;
   stack[stack_ptr] = _idx;
   s_cnt[stack_ptr] = _cnt;
   s_i[stack_ptr] = 0;

   stack_ptr++;
   int i = 0;
   while(stack_ptr > 0){
     for(;i < _cnt;){
       _vtx = treePtr->child_vtx[_idx + i];
       //lower = wtf.Get(_vtx);
       //upper = wtf.Get( _vtx + 1);
       vtkm::UInt8 type = ShapesPortal.Get(_vtx);
       vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);
       if(type == vtkm::CELL_SHAPE_HEXAHEDRON){
         lower = points.Get(cur_offset);
         upper = points.Get(cur_offset+1);
         ret = box(rayOrigin, rayDir, lower, upper);
         if (ret[0] > 0){
           //push onto stack
           stack[stack_ptr] = _idx;
           s_cnt[stack_ptr] = _cnt;
           s_i[stack_ptr] = i;


           _cnt = treePtr->child_cnt[_idx + i];
           _idx = treePtr->child_idx[_idx + i];
           _vtx = treePtr->child_vtx[_idx];
           i = 0;
           stack_ptr++;
         }
         else{
           i++;
         }
       }
       else{

         //reached a leaf
         switch(type){
         case vtkm::CELL_SHAPE_TRIANGLE:
           cyl_bottom = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));
           cyl_top = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset + 1));
           cyl_radius = vtkm::Float32(scalars.Get(cur_offset));
           //ret = vtkm::Vec<vtkm::Float32, 3>(1,1,1);
           ret = cylinder(rayOrigin, rayDir, cyl_bottom, cyl_top, cyl_radius);
           if (ret[0] > 0) {
             if (ret[1] < minDistance) {
               minDistance = ret[1];
               hitIndex = 35;
               fin_type = type;
               fin_offset = cur_offset;
             }
           }
           break;

         case vtkm::CELL_SHAPE_LINE:
           box_ll = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));
           box_ur = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset+1));
           ret = box(rayOrigin, rayDir, box_ll, box_ur);
           if (ret[0] > 0) {
             if (ret[1] < minDistance) {
               minDistance = ret[1];
               hitIndex = 35;
               fin_type = type;
               fin_offset = cur_offset;
               face = ret[2];
             }
           }
           break;

         case vtkm::CELL_SHAPE_VERTEX:
           center = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));//1.5;
           ret = sphere(rayOrigin, rayDir, center, vtkm::Float32(scalars.Get(cur_offset)));
           if (ret[0] > 0) {
             if (ret[1] < minDistance) {
               minDistance = ret[1];
               hitIndex = 35;
               fin_type = type;
               fin_offset = cur_offset;
               fin_center = center;
             }
           }
           break;
         }
         i++;
       }
     }
     if (i >= _cnt ){
       stack_ptr--;
       _idx = stack[stack_ptr];
       _cnt = s_cnt[stack_ptr];
       i = s_i[stack_ptr];
       i++;
     }
   }


}

#endif
