//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2015 Sandia Corporation.
//  Copyright 2015 UT-Battelle, LLC.
//  Copyright 2015 Los Alamos National Security.
//
//  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
//  the U.S. Government retains certain rights in this software.
//
//  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
//  Laboratory (LANL), the U.S. Government retains certain rights in
//  this software.
//============================================================================
#ifndef TriangleIntersector_h
#define TriangleIntersector_h
#include <cstring>
#include <vtkm/cont/ArrayHandle.h>
#include <vtkm/cont/ArrayHandleCompositeVector.h>
#include <vtkm/cont/Timer.h>
#include <vtkm/exec/ExecutionWholeArray.h>
#include <vtkm/rendering/raytracing/BoundingVolumeHierarchy.h>
#include "Ray.h"
#include <vtkm/worklet/DispatcherMapField.h>
#include <vtkm/worklet/WorkletMapField.h>
#include <iostream>

namespace {
  const static vtkm::Int32 END_FLAG2 = -1000000000;
  const static vtkm::Float32 EPSILON2 = 0.0001f;
}

template <typename DeviceAdapter>
class TriangleIntersector
{
public:
  typedef typename vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32,4> > Float4ArrayHandle;
  typedef typename vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Int32,2> > Int2Handle;
  typedef typename vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Int32,4> > Int4Handle;
  typedef typename Float4ArrayHandle::ExecutionTypes<DeviceAdapter>::PortalConst Float4ArrayPortal;
  typedef typename Int2Handle::ExecutionTypes<DeviceAdapter>::PortalConst Int2ArrayPortal;
  typedef typename Int4Handle::ExecutionTypes<DeviceAdapter>::PortalConst Int4ArrayPortal;

  class Intersector : public vtkm::worklet::WorkletMapField
  {
  private:

    bool Occlusion;
    vtkm::Float32 MaxDistance;
    Float4ArrayPortal FlatBVH;
    Int4ArrayPortal Leafs;
	const vtkm::cont::DynamicCellSet *Cells;
	vtkm::cont::ArrayHandle<vtkm::UInt8>::PortalConstControl ShapesPortal;
	vtkm::cont::ArrayHandle<vtkm::Id>::PortalConstControl OffsetsPortal;

    VTKM_EXEC_EXPORT
    vtkm::Float32 rcp(vtkm::Float32 f)  const { return 1.0f/f;}
    VTKM_EXEC_EXPORT
    vtkm::Float32 rcp_safe(vtkm::Float32 f) const { return rcp((fabs(f) < 1e-8f) ? 1e-8f : f); }
  public:
    VTKM_CONT_EXPORT
    Intersector(bool occlusion,
                vtkm::Float32 maxDistance,
                vtkm::rendering::raytracing::LinearBVH &bvh,
				const vtkm::cont::DynamicCellSet *cellset)
      : Occlusion(occlusion),
        MaxDistance(maxDistance),
        FlatBVH(bvh.FlatBVH.PrepareForInput( DeviceAdapter() )),
        Leafs( bvh.LeafNodes.PrepareForInput( DeviceAdapter() )),
		Cells(cellset)
    {
    vtkm::cont::CellSetExplicit<> cellSetExplicit = Cells->Cast<vtkm::cont::CellSetExplicit<> >();
		ShapesPortal = cellSetExplicit.GetShapesArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell()).GetPortalConstControl();
		//const vtkm::cont::ArrayHandle<vtkm::Int32> indices = cellSetExplicit.GetNumIndicesArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell());
		//vtkm::cont::ArrayHandle<vtkm::Id> conn = cellSetExplicit.GetConnectivityArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell());

		// We need to somehow force the data set to build the index offsets
		//vtkm::IdComponent c = indices.GetPortalControl().Get(0);
		vtkm::Vec< vtkm::Id, 3> forceBuildIndices;
		cellSetExplicit.GetIndices(0, forceBuildIndices);

		OffsetsPortal = cellSetExplicit.GetIndexOffsetArray(vtkm::TopologyElementTagPoint(), vtkm::TopologyElementTagCell()).GetPortalConstControl();

	}
    typedef void ControlSignature(FieldIn<>,
                                  FieldIn<>,
                                  FieldOut<>,
                                  FieldOut<>,
                                  FieldOut<>,
                                  FieldOut<>,

                                  WholeArrayIn<vtkm::rendering::raytracing::Vec3RenderingTypes>,
                                  WholeArrayIn<vtkm::rendering::raytracing::ScalarRenderingTypes>);
    typedef void ExecutionSignature(_1,
                                    _2,
                                    _3,
                                    _4,
                                    _5,
                                    _6,
                                    _7,
                                    _8);

	typedef vtkm::Vec<vtkm::Float32, 3> vec3;

	vec3 box(const vec3 &ray_start,
		const vec3 &ray,
		const vec3 &lb,
		const vec3 &rt) const
	{
		vec3 rayfrac;
		rayfrac[0] = 1.0 / ray[0];
		rayfrac[1] = 1.0 / ray[1];
		rayfrac[2] = 1.0 / ray[2];
		float t1 = (lb[0] - ray_start[0])*rayfrac[0];
		float t2 = (rt[0] - ray_start[0])*rayfrac[0];
		float t3 = (lb[1] - ray_start[1])*rayfrac[1];
		float t4 = (rt[1] - ray_start[1])*rayfrac[1];
		float t5 = (lb[2] - ray_start[2])*rayfrac[2];
		float t6 = (rt[2] - ray_start[2])*rayfrac[2];

		int face = 0;

		//float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
		float tmin = 0;
		if (t1 < t2) {
			tmin = t1;
			face = 0;
		}
		else {
			tmin = t2;
			face = 1;
		}
		if (t3 < t4) {
			if (tmin < t3) {
				tmin = t3;
				face = 2;
			}
		}
		else {
			if (tmin < t4) {
				tmin = t4;
				face = 3;
			}

		}
		if (t5 < t6) {
			if (tmin < t5) {
				tmin = t5;
				face = 4;
			}
		}
		else {
			if (tmin < t6) {
				tmin = t6;
				face = 5;
			}

		}
		float tmax = vtkm::Min(vtkm::Min(vtkm::Max(t1, t2), vtkm::Max(t3, t4)), vtkm::Max(t5, t6));

		// if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
		if (tmax < 0)
		{
			return vec3(0, tmax, 0);
		}

		// if tmin > tmax, ray doesn't intersect AABB
		if (tmin > tmax)
		{
			return vec3(0, tmax, 0);
		}

		return vec3(1, tmin, face);
	}
vec3 cylinder(const vec3 &ray_start,
		const vec3 &ray_direction,
		const vec3 &p,
		const vec3 &q,
		float r) const
	{
		float t = 0;
    vec3 ray_end = ray_start + ray_direction * 1000;
		vec3 d = q - p;
		vec3 m = ray_start - p;
    vec3 n = ray_end - ray_start;
		float md = dot(m, d);
    float nd = dot(n, d);
		float dd = dot(d, d);
		if ((md < 0.0f) && (md + nd < 0.0f)) return vec3(0, 0, 0); //segment outside 'p' side of cylinder
		if ((md > dd) && (md + nd  > dd))  return vec3(0, 0, 0); //segment outside 'q' side of cylinder
    float nn = dot(n, n);
    float mn = dot(m, n);
		float a = dd * nn - nd *nd;
		float k = dot(m, m) - r * r;
		float c = dd * k - md * md;

    if (fabs(a) < 1e-6) {
			if (c > 0.0) //'a' and thus the segment lie outside cylinder
				return vec3(0, 0, 0);
			if (md < 0.0f) //intesect segment against 'p' endcap 
				t = -mn / nn;
			else if (md > dd) //intersect segment against 'q' endcap
t = (nd - mn) / nn;
			else //else 'a' lies inside cylinder 
				t = 0;

				return vec3(1, 2, 0);
		}
		float b = dd * mn - nd * md;
		float discr = b * b - a * c;
		if (discr < 0.0f) //no roots 
			return vec3(0, 0, 0);
		t = (-b - sqrt(discr)) / a;
		if (md + t * nd < 0.0f) {
			//intersect outside cylinder on 'p' side
			if (nd <= 0.0f) return vec3(0.0, 0.0, 0);
			t = -md / nd;
			return vec3(k + 2 * t * (mn + t *nn) <= 0.0f, t, 0);
		}
		else if (md + t * nd > dd) {
			//intersect outside cylinder on 'q' side
			if (nd >= 0.0f) return vec3(0, 0, 0);
			t = (dd - md) / nd;

			return vec3(k + dd - 2 * md + t * (2 * (mn - nd) + t * nn) <= 0.0f, t, 0);
		}

		return vec3(1, t, 0);

	}

	vec3 sphere(const vec3 &ray_start,
		const vec3 &ray_direction,
		const vec3 &center,
		float r) const
	{
		vec3 p = ray_start - center;

		float b = dot(p, ray_direction);
		float c = dot(p, p) - r * r;

		if (c > 0 && b > 0) {
			return vec3(0, 0, 0);
		}

		float discr = b * b - c;

		if (discr < 0.0) {
			return vec3(0, 0, 0);
		}

		float t = -b - sqrt(discr);

		if (t < 0)
			return vec3(0, 0, 0);

		return vec3(1.0, t, 0);
	}
	template<typename PointPortalType, typename ScalarPortalType>
	VTKM_EXEC_EXPORT
		void operator()(const vtkm::Vec<vtkm::Float32, 3> &rayDir,
			const vtkm::Vec<vtkm::Float32, 3> &rayOrigin,
			vtkm::Float32 &distance,
			vtkm::Id &hitIndex,
			vtkm::Vec<vtkm::Float32, 3> &normal,
      vtkm::Float32 &scalar_out,
			const PointPortalType &points,
			const ScalarPortalType &scalars) const
	{
		float minDistance = MaxDistance;
		hitIndex = -1;
		float dirx = rayDir[0];
		float diry = rayDir[1];
		float dirz = rayDir[2];

		float invDirx = rcp_safe(dirx);
		float invDiry = rcp_safe(diry);
		float invDirz = rcp_safe(dirz);
		int currentNode;

		int todo[64];
		int stackptr = 0;
		int barrier = (int)END_FLAG2;
		currentNode = 0;

		todo[stackptr] = barrier;

		float originX = rayOrigin[0];
		float originY = rayOrigin[1];
		float originZ = rayOrigin[2];
		float originDirX = originX * invDirx;
		float originDirY = originY * invDiry;
		float originDirZ = originZ * invDirz;


		vec3 ret;
		vec3 cyl_top, cyl_bottom;
		vec3 center;
		vtkm::Float32 cyl_radius;
		vec3 box_ll, box_ur;

		vtkm::UInt8 fin_type = 0;
    vtkm::UInt8 face = 0;
		vtkm::Id fin_offset = 0;
		vec3 fin_center;

		for (int i = 0; i< ShapesPortal.GetNumberOfValues(); i++) {
			vtkm::UInt8 type = ShapesPortal.Get(i);
			vtkm::Id cur_offset = OffsetsPortal.Get(i);
			switch (type) {
      case vtkm::CELL_SHAPE_TRIANGLE:
        scalar_out = 0.4;
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

		}


	   if (minDistance < MaxDistance) {
       scalar_out = 1.0;
      vec3 pos = rayOrigin + rayDir * minDistance;
			vec3 pt;
			switch (fin_type) {
			//cylinder
			case vtkm::CELL_SHAPE_TRIANGLE:
				pt = vec3(points.Get(fin_offset)[0], pos[1], points.Get(fin_offset)[2]);
        normal = pos - pt;
        vtkm::Normalize(normal);
				break;

			//box
      case vtkm::CELL_SHAPE_LINE:
        scalar_out = 0.4;
        if (face == 0) {
          normal = vec3(-1.0, 0, 0);
				}
        else if (face == 1) {
          normal = vec3(1.0, 0, 0);
				}
        else if (face == 2) {
          normal = vec3(0.0, -1.0, 0);
				}
        else if (face == 3) {
          normal = vec3(0.0, 1.0, 0);
				}
        else if (face == 4) {
          normal = vec3(0.0, 0.0, -1.0);
				}
        else if (face == 5) {
          normal = vec3(0.0, 0.0, 1.0);
				}

				break;
			//sphere
			case vtkm::CELL_SHAPE_VERTEX:
        normal = pos - fin_center;
				vtkm::Normalize(normal);
				break;
			}
		}

	   //while(currentNode != END_FLAG2)
      //{
      //  if(currentNode>-1)
      //  {

      //    vtkm::Vec<vtkm::Float32,4> first4  = FlatBVH.Get(currentNode);
      //    vtkm::Vec<vtkm::Float32,4> second4 = FlatBVH.Get(currentNode+1);
      //    vtkm::Vec<vtkm::Float32,4> third4  = FlatBVH.Get(currentNode+2);
      //    bool hitLeftChild,hitRightChild;

      //    vtkm::Float32 xmin0 = first4[0] * invDirx - originDirX;
      //    vtkm::Float32 ymin0 = first4[1] * invDiry - originDirY;
      //    vtkm::Float32 zmin0 = first4[2] * invDirz - originDirZ;
      //    vtkm::Float32 xmax0 = first4[3] * invDirx - originDirX;
      //    vtkm::Float32 ymax0 = second4[0] * invDiry - originDirY;
      //    vtkm::Float32 zmax0 = second4[1] * invDirz - originDirZ;

      //    vtkm::Float32 min0 = vtkm::Max(vtkm::Max(vtkm::Max(vtkm::Min(ymin0,ymax0),vtkm::Min(xmin0,xmax0)),vtkm::Min(zmin0,zmax0)),0.f);
      //    vtkm::Float32 max0 = vtkm::Min(vtkm::Min(vtkm::Min(vtkm::Max(ymin0,ymax0),vtkm::Max(xmin0,xmax0)),vtkm::Max(zmin0,zmax0)), minDistance);
      //    hitLeftChild = ( max0 >= min0 );

      //    vtkm::Float32 xmin1 = second4[2] * invDirx - originDirX;
      //    vtkm::Float32 ymin1 = second4[3] * invDiry - originDirY;
      //    vtkm::Float32 zmin1 = third4[0] * invDirz - originDirZ;
      //    vtkm::Float32 xmax1 = third4[1] * invDirx - originDirX;
      //    vtkm::Float32 ymax1 = third4[2] * invDiry - originDirY;
      //    vtkm::Float32 zmax1 = third4[3] * invDirz - originDirZ;

      //    vtkm::Float32 min1 = vtkm::Max(vtkm::Max(vtkm::Max(vtkm::Min(ymin1,ymax1),vtkm::Min(xmin1,xmax1)),vtkm::Min(zmin1,zmax1)),0.f);
      //    vtkm::Float32 max1 = vtkm::Min(vtkm::Min(vtkm::Min(vtkm::Max(ymin1,ymax1),vtkm::Max(xmin1,xmax1)),vtkm::Max(zmin1,zmax1)), minDistance);
      //    hitRightChild = ( max1 >= min1 );

      //  if(!hitLeftChild && !hitRightChild)
      //  {
      //    currentNode = todo[stackptr];
      //    stackptr--;
      //  }
      //  else
      //  {
      //    vtkm::Vec<vtkm::Float32,4> children  = FlatBVH.Get(currentNode+3); //Children.Get(currentNode);
      //    vtkm::Int32 leftChild;
      //    memcpy(&leftChild, &children[0],4);
      //    vtkm::Int32 rightChild;
      //    memcpy(&rightChild, &children[1],4);
      //    currentNode = (hitLeftChild) ? leftChild : rightChild;
      //    if(hitLeftChild && hitRightChild)
      //    {
      //      if(min0 > min1)
      //      {
      //        currentNode = rightChild;
      //        stackptr++;
      //        todo[stackptr] = leftChild;
      //      }
      //      else
      //      {
      //        stackptr++;
      //        todo[stackptr] = rightChild;
      //      }
      //    }
      //  }
      //} // if inner node

      //if(currentNode < 0 && currentNode != barrier)//check register usage
      //{
      //  currentNode = -currentNode - 1; //swap the neg address
      //  vtkm::Vec<vtkm::Int32, 4> leafnode = Leafs.Get(currentNode);
      //  vtkm::Vec<vtkm::Float32, 3> a = vtkm::Vec<vtkm::Float32,3>(points.Get(leafnode[1]));
      //  vtkm::Vec<vtkm::Float32, 3> b = vtkm::Vec<vtkm::Float32,3>(points.Get(leafnode[2]));
      //  vtkm::Vec<vtkm::Float32, 3> c = vtkm::Vec<vtkm::Float32,3>(points.Get(leafnode[3]));

      //  vtkm::Vec<vtkm::Float32, 3> e1 = b - a;
      //  vtkm::Vec<vtkm::Float32, 3> e2 = c - a;


      //  vtkm::Vec<vtkm::Float32, 3> p;
      //  p[0] = diry * e2[2] - dirz * e2[1];
      //  p[1] = dirz * e2[0] - dirx * e2[2];
      //  p[2] = dirx * e2[1] - diry * e2[0];
      //  vtkm::Float32 dot = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
      //  if(dot != 0.f)
      //  {
      //    dot = 1.f / dot;
      //    vtkm::Vec<vtkm::Float32, 3> t;
      //    t[0] = originX - a[0];
      //    t[1] = originY - a[1];
      //    t[2] = originZ - a[2];

      //    float u = (t[0]* p[0] + t[1] * p[1] + t[2] * p[2]) * dot;
      //    if(u >= (0.f - EPSILON2) && u <= (1.f + EPSILON2))
      //    {

      //      vtkm::Vec<vtkm::Float32, 3> q; // = t % e1;
      //      q[0] = t[1] * e1[2] - t[2] * e1[1];
      //      q[1] = t[2] * e1[0] - t[0] * e1[2];
      //      q[2] = t[0] * e1[1] - t[1] * e1[0];
      //      vtkm::Float32 v = (dirx * q[0] + diry * q[1] + dirz * q[2]) * dot;

      //      if(v >= (0.f - EPSILON2) && v <= (1.f + EPSILON2))
      //      {

      //        vtkm::Float32 dist = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * dot;
      //        if((dist > EPSILON2 && dist < minDistance) && !(u + v > 1) )
      //        {
      //          minDistance = dist;
      //          hitIndex = currentNode;
      //          minU = u;
      //          minV = v;
      //          if(Occlusion) return;//or set todo to -1
      //        }
      //      }
      //    }
      //  }

    //    currentNode = todo[stackptr];
    //    stackptr--;
    //  } // if leaf node

    //} //while
    distance = minDistance;

  }// ()
};



  VTKM_CONT_EXPORT
  void run(Ray<DeviceAdapter> &rays,
    vtkm::rendering::raytracing::LinearBVH &bvh,
           const vtkm::cont::DynamicCellSet *cells,
           vtkm::cont::DynamicArrayHandleCoordinateSystem coordsHandle,
           const vtkm::cont::Field *scalarField)
  {
    vtkm::worklet::DispatcherMapField< Intersector >( Intersector( false, 10000000.f, bvh, cells) )
      .Invoke( rays.Dir,
               rays.Origin,
               rays.Distance,
//               rays.U,
//               rays.V,
               rays.HitIdx,
			   rays.Normal,
               rays.Scalar,
               coordsHandle,
               *scalarField);
  }

}; // class intersector

#endif //vtk_m_rendering_raytracing_TriagnleIntersector_h
