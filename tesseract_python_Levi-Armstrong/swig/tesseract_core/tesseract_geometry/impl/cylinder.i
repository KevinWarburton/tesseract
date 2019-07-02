%{
#include <tesseract_geometry/impl/cylinder.h>
%}

%shared_ptr(tesseract_geometry::Cylinder) //overloaded with const Box

//TEST FOR AUTO UPDATE FUNCTIONS
//%include <tesseract_geometry/impl/cylinder.h>

namespace tesseract_geometry
{
  class Cylinder;
  typedef std::shared_ptr<Cylinder> CylinderPtr;
  typedef std::shared_ptr<const Cylinder> CylinderConstPtr;

  class Cylinder : public Geometry
  {
  public:
    Cylinder(double r, double l) : Geometry(GeometryType::CYLINDER), r_(r), l_(l) {}
    ~Cylinder() override = default;

    double getRadius() const { return r_; }
    double getLength() const { return l_; }

    GeometryPtr clone() const override { return CylinderPtr(new Cylinder(r_, l_)); }

  private:
    double r_;
    double l_;
  };
}
