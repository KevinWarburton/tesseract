%{
#include <tesseract_geometry/impl/octree.h>
%}

%shared_ptr(tesseract_geometry::Octree)
%shared_ptr(octomap::OcTree)

namespace tesseract_geometry
{
  class Octree;
  typedef std::shared_ptr<Octree> OctreePtr;
  typedef std::shared_ptr<const Octree> OctreeConstPtr;

  class Octree : public Geometry
  {
  public:
    enum SubType
    {
      BOX,
      SPHERE_INSIDE,
      SPHERE_OUTSIDE
    };

    Octree(const std::shared_ptr<const octomap::OcTree>& octree, SubType sub_type)  : Geometry(GeometryType::OCTREE), octree_(octree), sub_type_(sub_type) {}
    ~Octree() override = default;

    const std::shared_ptr<const octomap::OcTree>& getOctree() const { return octree_; }
    SubType getSubType() const { return sub_type_; }

    GeometryPtr clone() const override { return OctreePtr(new Octree(octree_, sub_type_)); }

    /**
     * @brief Octrees are typically generated from 3D sensor data so this method
     * should be used to efficiently update the collision shape.
     */
    void update() { assert(false); }

  private:
    std::shared_ptr<const octomap::OcTree> octree_;
    SubType sub_type_;
  };
}
