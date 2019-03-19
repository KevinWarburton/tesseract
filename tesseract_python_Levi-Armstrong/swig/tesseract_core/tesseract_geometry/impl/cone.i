%{
#include <tesseract_geometry/impl/cone.h>
%}

%shared_ptr(tesseract_geometry::Cone)

//TEST FOR AUTO UPDATE FUNCTIONS
////%include <tesseract_geometry/macros.h>
////%include <memory>
////%include <tesseract_geometry/geometry.h>
//%include <tesseract_geometry/impl/cone.h>

namespace tesseract_geometry
{
  class Cone;
  typedef std::shared_ptr<Cone> ConePtr;
  typedef std::shared_ptr<const Cone> ConeConstPtr;

  class Cone : public Geometry
  {
  public:
    Cone(double r, double l) : Geometry(GeometryType::CONE), r_(r), l_(l) {}
    ~Cone() override = default;

    double getRadius() const { return r_; }
    double getLength() const { return l_; }

    GeometryPtr clone() const override { return ConePtr(new Cone(r_, l_)); }

  private:
    double r_;
    double l_;
  };

}
