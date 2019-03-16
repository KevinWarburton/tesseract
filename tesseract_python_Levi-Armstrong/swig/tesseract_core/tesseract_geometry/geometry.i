%{
//#include <tesseract_core/tesseract_geometry/include/tesseract_geometry/impl/box.h>
//#include "../../../../tesseract_core/tesseract_geometry/include/tesseract_geometry/geometry.h"
//#include <tesseract_geometry/include/tesseract_geometry/geometry.h>
#include <tesseract_geometry/geometry.h>
//#include "/home/ros-industrial/repos_KevinWarburton/tesseract_python_wrapping_Levi-Armstrong_version/src/tesseract/tesseract_core/tesseract_geometry/include/tesseract_geometry/impl/box.h"
%}

%shared_ptr(tesseract_geometry::Geometry)

namespace tesseract_geometry
{
  enum GeometryType {SPHERE, CYLINDER, CONE, BOX, PLANE, MESH, CONVEX_MESH, SDF_MESH, OCTREE};

  class Geometry;
  typedef std::shared_ptr<Geometry> GeometryPtr;
  typedef std::shared_ptr<const Geometry> GeometryConstPtr;
  typedef std::vector<GeometryPtr> Geometrys;
  typedef std::vector<GeometryConstPtr> GeometrysConst;

  class Geometry
  {
  public:
    explicit Geometry(GeometryType type) : type_(type) {}
    virtual ~Geometry() = default;

    /** \brief Create a copy of this shape */
    virtual GeometryPtr clone() const = 0;

    GeometryType getType() const { return type_; }

  private:
    /** \brief The type of the shape */
    GeometryType type_;
  };
}
