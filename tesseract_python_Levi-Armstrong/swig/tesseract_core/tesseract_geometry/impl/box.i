%{
#include <tesseract_geometry/impl/box.h>
%}

%shared_ptr(tesseract_geometry::Box) //overloaded with const Box

//TEST FOR AUTO UPDATE FUNCTIONS
////%include <tesseract_geometry/macros.h>
////%include <memory>
////%include <tesseract_geometry/geometry.h>
//%include <tesseract_geometry/impl/box.h>

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
