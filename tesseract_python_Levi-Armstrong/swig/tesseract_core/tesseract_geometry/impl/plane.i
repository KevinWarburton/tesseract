%{
#include <tesseract_geometry/impl/plane.h>
%}

%shared_ptr(tesseract_geometry::Plane)
%shared_ptr(octomap::OcTree)

namespace tesseract_geometry
{
  class Plane;
  typedef std::shared_ptr<Plane> PlanePtr;

  class Plane : public Geometry
  {
  public:
    Plane(double a, double b, double c, double d) : Geometry(GeometryType::PLANE), a_(a), b_(b), c_(c), d_(d) {}
    ~Plane() override = default;

    double getA() const { return a_; }
    double getB() const { return b_; }
    double getC() const { return c_; }
    double getD() const { return d_; }

    GeometryPtr clone() const override { return PlanePtr(new Plane(a_, b_, c_, d_)); }

  private:
    double a_;
    double b_;
    double c_;
    double d_;
  };
}
