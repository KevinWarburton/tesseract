%{
//#include <tesseract_core/tesseract_geometry/include/tesseract_geometry/impl/box.h>
//#include "/home/ros-industrial/repos_KevinWarburton/tesseract_python_wrapping_Levi-Armstrong_version/src/tesseract/tesseract_core/tesseract_geometry/include/tesseract_geometry/impl/box.h"
//#include <tesseract_geometry/include/tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/geometry.h>
//#include "../../../../../tesseract_core/tesseract_geometry/include/tesseract_geometry/geometry.h"
%}

%shared_ptr(tesseract_geometry::Box) //overloaded with const Box

namespace tesseract_geometry
{
  class Box;
  typedef std::shared_ptr<Box> BoxPtr;
  typedef std::shared_ptr<const Box> BoxConstPtr;

  class Box : public Geometry
  {
  public:
    Box(double x, double y, double z) : Geometry(GeometryType::BOX), x_(x), y_(y), z_(z) {}
    ~Box() override = default;

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getZ() const { return z_; }

    GeometryPtr clone() const override { return BoxPtr(new Box(x_, y_, z_)); }

  private:
    double x_;
    double y_;
    double z_;
  };
}
