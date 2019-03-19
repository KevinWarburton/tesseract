%{
#include <tesseract_geometry/types.h>
%}

//Define template macros for data types and define data types
%define tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%template(string_pair) std::pair<std::string,std::string>;


tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
tesseract_aligned_vector(VectorVector3d, Eigen::Vector3d);
tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);

tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);



//TEST FOR AUTO UPDATE FUNCTIONS
////%include <tesseract_geometry/macros.h>
////%include <memory>
////%include <tesseract_geometry/geometry.h>
//%include <tesseract_geometry/types.h>

namespace tesseract_geometry
{
  template <typename T>
  using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

  template <typename Key, typename Value>
  using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

  template <typename Key, typename Value>
  using AlignedUnorderedMap = std::unordered_map<Key,
                                                 Value,
                                                 std::hash<Key>,
                                                 std::equal_to<Key>,
                                                 Eigen::aligned_allocator<std::pair<const Key, Value>>>;

  using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
  using VectorVector4d = AlignedVector<Eigen::Vector4d>;
  using VectorVector3d = std::vector<Eigen::Vector3d>;
  using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;
}
