%{
#include <tesseract_geometry/impl/convex_mesh.h>
%}

%shared_ptr(tesseract_geometry::ConvexMesh)
//%shared_ptr(tesseract_geometry::VectorVector3d)
//%shared_ptr(tesseract_geometry::Eigen::VectorXi)

//TEST FOR AUTO UPDATE FUNCTIONS
////%include <tesseract_geometry/macros.h>
////%include <memory>
////%include <tesseract_geometry/geometry.h>
//%include <tesseract_geometry/impl/convex_mesh.h>

namespace tesseract_geometry
{
  class ConvexMesh;
  typedef std::shared_ptr<ConvexMesh> ConvexMeshPtr;
  typedef std::shared_ptr<const ConvexMesh> ConvexMeshConstPtr;

  class ConvexMesh : public Geometry
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConvexMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& faces) : Geometry(GeometryType::CONVEX_MESH), vertices_(vertices), faces_(faces)
    {
      vertice_count_ = static_cast<int>(vertices->size());

      face_count_ = 0;
      for (int i = 0; i < faces_->size(); ++i)
      {
        ++face_count_;
        int num_verts = (*faces_)(i);
        i += num_verts;
      }

    }

    ConvexMesh(const std::shared_ptr<const VectorVector3d>& vertices, const std::shared_ptr<const Eigen::VectorXi>& faces, int face_count) : Geometry(GeometryType::CONVEX_MESH), vertices_(vertices), faces_(faces), face_count_(face_count)
    {
      vertice_count_ = static_cast<int>(vertices->size());
    }

    ~ConvexMesh() override = default;

    const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }
    const std::shared_ptr<const Eigen::VectorXi>& getFaces() const { return faces_; }

    int getVerticeCount() const { return vertice_count_; }
    int getFaceCount() const { return face_count_; }

    GeometryPtr clone() const override { return ConvexMeshPtr(new ConvexMesh(vertices_, faces_, face_count_)); }

  private:
    std::shared_ptr<const VectorVector3d> vertices_;
    std::shared_ptr<const Eigen::VectorXi> faces_;

    int vertice_count_;
    int face_count_;
  };
}
