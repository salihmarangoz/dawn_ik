#ifndef __COLLISION_SHAPES_H__
#define __COLLISION_SHAPES_H__

namespace salih_marangoz_thesis
{

enum CommonShapeType : int64_t // Golomb_ruler: 0 1 6 10 23 26 34 41 53 55
{
  SPHERE    = 0, // x,y,z,radius
  AXISPLANE = 1, // x,y,z
  PLANE     = 6  // x,y,z,nx,ny,nz     see: https://mathworld.wolfram.com/Plane.html
}

// 8*8 = 64 byte
struct CommonShape
{
  int64_t type;
  double params[7]; // TODO: hardcoded
}

template <typename T>
T computeDistance(int64_t type_a, const T* shape_a, int64_t type_b, const T* shape_b)
{
  // same shape
  if (type_a == type_b)
  {
    switch (type_a)
    {
      case SPHERE: return computeDistanceSphere2Sphere(shape_a, shape_b);
      //case AXISPLANE:
      //case PLANE:
    }
  }

  // reverse order
  if (type_a > type_b) return computeDistance(type_b, shape_b, type_a, shape_a);

  // between shapes
  switch (type_a - type_b)
  {
    case SPHERE-AXISPLANE: return computeDistanceSphere2Axisplane(shape_a, shape_b);
    case SPHERE-PLANE: return computeDistanceSphere2Plane(shape_a, shape_b);
    //case AXISPLANE-PLANE:
  }

  // not implemented error
}

template <typename T>
T computeDistanceSphere2Sphere(const T* shape_a, const T* shape_b)
{

}

template <typename T>
T computeDistanceSphere2Axisplane(const T* shape_a, const T* shape_b)
{
  
}

template <typename T>
T computeDistanceSphere2Plane(const T* shape_a, const T* shape_b)
{
  
}


} // namespace salih_marangoz_thesis

#endif // __COLLISION_SHAPES_H__