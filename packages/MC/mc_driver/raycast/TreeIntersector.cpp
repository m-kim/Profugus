#include "TreeIntersector.h"

Tree::vec3 TreeIntersector::cylinder(const vec3 &ray_start,
    const vec3 &ray_direction,
    const vec3 &p,
    const vec3 &q,
    float r)
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

Tree::vec3 TreeIntersector::sphere(const vec3 &ray_start,
    const vec3 &ray_direction,
    const vec3 &center,
    float r)
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

Tree::vec3 TreeIntersector::box(const vec3 &ray_start,
               const vec3 &ray,
               const vec3 &lb,
               const vec3 &rt)
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


