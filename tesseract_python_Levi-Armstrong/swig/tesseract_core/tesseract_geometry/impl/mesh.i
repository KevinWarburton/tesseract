%{
#include <tesseract_geometry/impl/mesh.h>
%}

%shared_ptr(tesseract_geometry::Mesh)

//TEST FOR AUTO UPDATE FUNCTIONS
//%include <tesseract_geometry/impl/cylinder.h>

namespace tesseract_geometry
{
  class Mesh;
  typedef std::shared_ptr<Mesh> MeshPtr;
  typedef std::shared_ptr<const Mesh> MeshConstPtr;

  class Mesh : public Geometry
  {
  public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Mesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& triangles) : Geometry(GeometryType::MESH), vertices_(vertices), triangles_(triangles)
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

    Mesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& triangles, int triangle_count) : Geometry(GeometryType::MESH), vertices_(vertices), triangles_(triangles), triangle_count_(triangle_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
      assert((triangle_count * 4) == triangles_->size());
    }

    ~Mesh() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }
    const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

    int getVerticeCount() const { return vertice_count_; }
    int getTriangleCount() const { return triangle_count_; }

    GeometryPtr clone() const override { return MeshPtr(new Mesh(vertices_, triangles_, triangle_count_)); }

  private:
    std::shared_ptr<const VectorVector3d> vertices_;
    std::shared_ptr<const Eigen::VectorXi> triangles_;
    int vertice_count_;
    int triangle_count_;
  };
}
