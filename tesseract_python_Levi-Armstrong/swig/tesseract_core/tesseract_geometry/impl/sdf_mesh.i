%{
#include <tesseract_geometry/impl/sdf_mesh.h>
%}

%shared_ptr(tesseract_geometry::SDFMesh)

namespace tesseract_geometry
{
  class SDFMesh;
  typedef std::shared_ptr<SDFMesh> SDFMeshPtr;
  typedef std::shared_ptr<const SDFMesh> SDFMeshConstPtr;

  class SDFMesh : public Geometry
  {
  public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SDFMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& triangles) : Geometry(GeometryType::SDF_MESH), vertices_(vertices), triangles_(triangles)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      triangle_count_ = 0;
      for (int i = 0; i < triangles_->size(); ++i)
      {
        ++triangle_count_;
        int num_verts = (*triangles_)(i);
        i += num_verts;
        assert(num_verts == 3);
      }
    }

    SDFMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& triangles, int triangle_count) : Geometry(GeometryType::SDF_MESH), vertices_(vertices), triangles_(triangles), triangle_count_(triangle_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
      assert((triangle_count * 4) == triangles_->size());
    }

    ~SDFMesh() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }
    const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

    int getVerticeCount() const { return vertice_count_; }
    int getTriangleCount() const { return triangle_count_; }

    GeometryPtr clone() const override { return SDFMeshPtr(new SDFMesh(vertices_, triangles_, triangle_count_)); }

    private:
      std::shared_ptr<const VectorVector3d> vertices_;
      std::shared_ptr<const Eigen::VectorXi> triangles_;

      int vertice_count_;
      int triangle_count_;
  };
}
