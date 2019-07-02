%{
#include <tesseract_geometry/impl/sphere.h>
%}

%shared_ptr(tesseract_geometry::Sphere)

namespace tesseract_geometry
{
  class Sphere;
  typedef std::shared_ptr<Sphere> SpherePtr;
  typedef std::shared_ptr<const Sphere> SphereConstPtr;

  class Sphere : public Geometry
  {
  public:
    explicit Sphere(double r) : Geometry(GeometryType::SPHERE), r_(r) {}
    ~Sphere() override = default;

    double getRadius() const { return r_; }

    GeometryPtr clone() const override { return SpherePtr(new Sphere(r_)); }

  private:
    double r_;
  };
}
