#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_geometry/utils.h>
#include <tesseract_geometry/impl/octree_utils.h>
#include <tesseract_common/utils.h>

TEST(TesseractGeometryUnit, Instantiation)  // NOLINT
{
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices =
      std::make_shared<const tesseract_common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> faces = std::make_shared<const Eigen::VectorXi>();

  auto box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);
  auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);
  auto capsule = std::make_shared<tesseract_geometry::Capsule>(1, 1);
  auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);
  auto sphere = std::make_shared<tesseract_geometry::Sphere>(1);
  auto convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(vertices, faces);
  auto mesh = std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  auto sdf_mesh = std::make_shared<tesseract_geometry::SDFMesh>(vertices, faces);
  auto octree = std::make_shared<tesseract_geometry::Octree>(nullptr, tesseract_geometry::OctreeSubType::BOX);
  auto compound_mesh =
      std::make_shared<tesseract_geometry::CompoundMesh>(std::vector<std::shared_ptr<tesseract_geometry::PolygonMesh>>{
          std::make_shared<tesseract_geometry::Mesh>(vertices, faces),
          std::make_shared<tesseract_geometry::Mesh>(vertices, faces) });

  // Instead making this depend on pcl it expects the structure to have a member called points which is a vector
  // of another object with has float members x, y and z.
  struct TestPointCloud
  {
    struct point
    {
      double x;
      double y;
      double z;
    };

    std::vector<point> points;
  };

  TestPointCloud pc;
  auto co = tesseract_geometry::createOctree(pc, 0.01, false);
  auto octree_pc =
      std::make_shared<tesseract_geometry::Octree>(std::move(co), tesseract_geometry::OctreeSubType::BOX, false);
}

TEST(TesseractGeometryUnit, Box)  // NOLINT
{
  using T = tesseract_geometry::Box;
  auto geom = std::make_shared<T>(1, 1, 1);
  EXPECT_NEAR(geom->getX(), 1, 1e-5);
  EXPECT_NEAR(geom->getY(), 1, 1e-5);
  EXPECT_NEAR(geom->getZ(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::BOX);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getX(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getY(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getZ(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::BOX);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Box(2, 1, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Box(1, 2, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Box(1, 1, 2)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Cone(1, 1)));
}

TEST(TesseractGeometryUnit, Cone)  // NOLINT
{
  using T = tesseract_geometry::Cone;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::CONE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::CONE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Cone(2, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Cone(1, 2)));
}

TEST(TesseractGeometryUnit, Cylinder)  // NOLINT
{
  using T = tesseract_geometry::Cylinder;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::CYLINDER);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::CYLINDER);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Cylinder(2, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Cylinder(1, 2)));
}

TEST(TesseractGeometryUnit, Capsule)  // NOLINT
{
  using T = tesseract_geometry::Capsule;
  auto geom = std::make_shared<T>(1, 1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_NEAR(geom->getLength(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::CAPSULE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getLength(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::CAPSULE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Capsule(2, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Capsule(1, 2)));
}

TEST(TesseractGeometryUnit, Sphere)  // NOLINT
{
  using T = tesseract_geometry::Sphere;
  auto geom = std::make_shared<T>(1);
  EXPECT_NEAR(geom->getRadius(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::SPHERE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getRadius(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::SPHERE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Sphere(2)));
}

TEST(TesseractGeometryUnit, Plane)  // NOLINT
{
  using T = tesseract_geometry::Plane;
  auto geom = std::make_shared<T>(1, 1, 1, 1);
  EXPECT_NEAR(geom->getA(), 1, 1e-5);
  EXPECT_NEAR(geom->getB(), 1, 1e-5);
  EXPECT_NEAR(geom->getC(), 1, 1e-5);
  EXPECT_NEAR(geom->getD(), 1, 1e-5);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::PLANE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getA(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getB(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getC(), 1, 1e-5);
  EXPECT_NEAR(std::static_pointer_cast<T>(geom_clone)->getD(), 1, 1e-5);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::PLANE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Plane(2, 1, 1, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Plane(1, 2, 1, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Plane(1, 1, 2, 1)));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::Plane(1, 1, 1, 2)));
}

TEST(TesseractGeometryUnit, PolygonMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(5);
  (*faces)(0) = 4;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;
  (*faces)(4) = 3;

  using T = tesseract_geometry::PolygonMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVertexCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 1);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::POLYGON_MESH);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 1);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::POLYGON_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(
      *geom,
      tesseract_geometry::PolygonMesh(std::make_shared<tesseract_common::VectorVector3d>(),
                                      std::make_shared<Eigen::VectorXi>())));
}

TEST(TesseractGeometryUnit, ConvexMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(5);
  (*faces)(0) = 4;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;
  (*faces)(4) = 3;

  using T = tesseract_geometry::ConvexMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVertexCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 1);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::CONVEX_MESH);
  EXPECT_EQ(geom->getCreationMethod(), tesseract_geometry::ConvexMesh::CreationMethod::DEFAULT);
  geom->setCreationMethod(tesseract_geometry::ConvexMesh::CreationMethod::CONVERTED);
  EXPECT_EQ(geom->getCreationMethod(), tesseract_geometry::ConvexMesh::CreationMethod::CONVERTED);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 1);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::CONVEX_MESH);
  EXPECT_EQ(geom->getCreationMethod(), tesseract_geometry::ConvexMesh::CreationMethod::CONVERTED);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(
      *geom,
      tesseract_geometry::ConvexMesh(std::make_shared<tesseract_common::VectorVector3d>(),
                                     std::make_shared<Eigen::VectorXi>())));
}

TEST(TesseractGeometryUnit, CompoundConvexMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(5);
  (*faces)(0) = 4;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;
  (*faces)(4) = 3;

  using T = tesseract_geometry::ConvexMesh;
  auto sub_geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(sub_geom->getVertices() != nullptr);
  EXPECT_TRUE(sub_geom->getFaces() != nullptr);
  EXPECT_TRUE(sub_geom->getVertexCount() == 4);
  EXPECT_TRUE(sub_geom->getFaceCount() == 1);
  EXPECT_EQ(sub_geom->getType(), tesseract_geometry::GeometryType::CONVEX_MESH);
  EXPECT_FALSE(sub_geom->getUUID().is_nil());

  std::vector<tesseract_geometry::PolygonMesh::Ptr> poly_meshes;
  poly_meshes.push_back(sub_geom);
  poly_meshes.push_back(sub_geom);
  poly_meshes.push_back(sub_geom);

  auto geom = std::make_shared<tesseract_geometry::CompoundMesh>(poly_meshes);
  EXPECT_EQ(geom->getMeshes().size(), 3);
  EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);

  auto geom_clone = std::static_pointer_cast<tesseract_geometry::CompoundMesh>(geom->clone());
  EXPECT_EQ(geom_clone->getMeshes().size(), 3);
  EXPECT_EQ(geom_clone->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(
      tesseract_common::almostEqualRelativeAndAbs(geom_clone->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::CompoundMesh()));

  {  // Test convex hull constructors
    std::vector<tesseract_geometry::ConvexMesh::Ptr> convex_meshes;
    convex_meshes.push_back(sub_geom);
    convex_meshes.push_back(sub_geom);
    convex_meshes.push_back(sub_geom);

    auto geom = std::make_shared<tesseract_geometry::CompoundMesh>(convex_meshes);
    EXPECT_EQ(geom->getMeshes().size(), 3);
    EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
    EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  }

  {  // Test convex hull constructors
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
    vertices->emplace_back(1, 1, 0);
    vertices->emplace_back(1, -1, 0);
    vertices->emplace_back(-1, -1, 0);
    vertices->emplace_back(1, -1, 0);

    auto faces = std::make_shared<Eigen::VectorXi>();
    faces->resize(8);
    (*faces)(0) = 3;
    (*faces)(1) = 0;
    (*faces)(2) = 1;
    (*faces)(3) = 2;

    (*faces)(4) = 3;
    (*faces)(5) = 0;
    (*faces)(6) = 2;
    (*faces)(7) = 3;

    using T = tesseract_geometry::SDFMesh;
    auto sub_geom = std::make_shared<T>(vertices, faces);
    EXPECT_TRUE(sub_geom->getVertices() != nullptr);
    EXPECT_TRUE(sub_geom->getFaces() != nullptr);
    EXPECT_TRUE(sub_geom->getVertexCount() == 4);
    EXPECT_TRUE(sub_geom->getFaceCount() == 2);
    EXPECT_EQ(sub_geom->getType(), tesseract_geometry::GeometryType::SDF_MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    std::vector<tesseract_geometry::SDFMesh::Ptr> sdf_meshes;
    sdf_meshes.push_back(sub_geom);
    sdf_meshes.push_back(sub_geom);
    sdf_meshes.push_back(sub_geom);

    auto geom = std::make_shared<tesseract_geometry::CompoundMesh>(sdf_meshes);
    EXPECT_EQ(geom->getMeshes().size(), 3);
    EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
    EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  }

  {  // Test convex hull constructors
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
    vertices->emplace_back(1, 1, 0);
    vertices->emplace_back(1, -1, 0);
    vertices->emplace_back(-1, -1, 0);
    vertices->emplace_back(1, -1, 0);

    auto faces = std::make_shared<Eigen::VectorXi>();
    faces->resize(8);
    (*faces)(0) = 3;
    (*faces)(1) = 0;
    (*faces)(2) = 1;
    (*faces)(3) = 2;

    (*faces)(4) = 3;
    (*faces)(5) = 0;
    (*faces)(6) = 2;
    (*faces)(7) = 3;

    using T = tesseract_geometry::Mesh;
    auto sub_geom = std::make_shared<T>(vertices, faces);
    EXPECT_TRUE(sub_geom->getVertices() != nullptr);
    EXPECT_TRUE(sub_geom->getFaces() != nullptr);
    EXPECT_TRUE(sub_geom->getVertexCount() == 4);
    EXPECT_TRUE(sub_geom->getFaceCount() == 2);
    EXPECT_EQ(sub_geom->getType(), tesseract_geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    std::vector<tesseract_geometry::Mesh::Ptr> meshes;
    meshes.push_back(sub_geom);
    meshes.push_back(sub_geom);
    meshes.push_back(sub_geom);

    auto geom = std::make_shared<tesseract_geometry::CompoundMesh>(meshes);
    EXPECT_EQ(geom->getMeshes().size(), 3);
    EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
    EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  }
}

TEST(TesseractGeometryUnit, Mesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces)(0) = 3;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;

  (*faces)(4) = 3;
  (*faces)(5) = 0;
  (*faces)(6) = 2;
  (*faces)(7) = 3;

  using T = tesseract_geometry::Mesh;

  {
    auto geom = std::make_shared<T>(vertices, faces);
    EXPECT_TRUE(geom->getVertices() != nullptr);
    EXPECT_TRUE(geom->getFaces() != nullptr);
    EXPECT_TRUE(geom->getVertexCount() == 4);
    EXPECT_TRUE(geom->getFaceCount() == 2);
    EXPECT_TRUE(geom->getMaterial() == nullptr);
    EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    auto geom_clone = geom->clone();
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 2);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getMaterial() == nullptr);
    EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::MESH);
    EXPECT_FALSE(geom_clone->getUUID().is_nil());
    EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

    geom->setUUID(geom_clone->getUUID());
    EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

    // Test isIdentical
    EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
    EXPECT_FALSE(
        tesseract_geometry::isIdentical(*geom,
                                        tesseract_geometry::Mesh(std::make_shared<tesseract_common::VectorVector3d>(),
                                                                 std::make_shared<Eigen::VectorXi>())));
  }

  {
    auto mat = std::make_shared<tesseract_geometry::MeshMaterial>();
    auto geom = std::make_shared<T>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), nullptr, nullptr, mat);
    EXPECT_TRUE(geom->getVertices() != nullptr);
    EXPECT_TRUE(geom->getFaces() != nullptr);
    EXPECT_TRUE(geom->getVertexCount() == 4);
    EXPECT_TRUE(geom->getFaceCount() == 2);
    EXPECT_TRUE(geom->getMaterial() != nullptr);
    EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::MESH);
    EXPECT_FALSE(geom->getUUID().is_nil());

    auto geom_clone = geom->clone();
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 2);
    EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getMaterial() != nullptr);
    EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::MESH);
    EXPECT_FALSE(geom_clone->getUUID().is_nil());
    EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

    geom->setUUID(geom_clone->getUUID());
    EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

    // Test isIdentical
    EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
    EXPECT_FALSE(
        tesseract_geometry::isIdentical(*geom,
                                        tesseract_geometry::Mesh(std::make_shared<tesseract_common::VectorVector3d>(),
                                                                 std::make_shared<Eigen::VectorXi>())));
  }
}

TEST(TesseractGeometryUnit, CompoundMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces)(0) = 3;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;

  (*faces)(4) = 3;
  (*faces)(5) = 0;
  (*faces)(6) = 2;
  (*faces)(7) = 3;

  using T = tesseract_geometry::Mesh;

  auto sub_geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(sub_geom->getVertices() != nullptr);
  EXPECT_TRUE(sub_geom->getFaces() != nullptr);
  EXPECT_TRUE(sub_geom->getVertexCount() == 4);
  EXPECT_TRUE(sub_geom->getFaceCount() == 2);
  EXPECT_TRUE(sub_geom->getMaterial() == nullptr);
  EXPECT_EQ(sub_geom->getType(), tesseract_geometry::GeometryType::MESH);
  EXPECT_FALSE(sub_geom->getUUID().is_nil());

  std::vector<tesseract_geometry::PolygonMesh::Ptr> meshes;
  meshes.push_back(sub_geom);
  meshes.push_back(sub_geom);
  meshes.push_back(sub_geom);

  auto geom = std::make_shared<tesseract_geometry::CompoundMesh>(meshes);
  EXPECT_EQ(geom->getMeshes().size(), 3);
  EXPECT_EQ(geom->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(geom->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = std::static_pointer_cast<tesseract_geometry::CompoundMesh>(geom->clone());
  EXPECT_EQ(geom_clone->getMeshes().size(), 3);
  EXPECT_EQ(geom_clone->getResource(), geom->getMeshes().front()->getResource());
  EXPECT_TRUE(
      tesseract_common::almostEqualRelativeAndAbs(geom_clone->getScale(), geom->getMeshes().front()->getScale()));
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::COMPOUND_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test Failure
  meshes.clear();
  meshes.push_back(sub_geom);
  EXPECT_ANY_THROW(std::make_shared<tesseract_geometry::CompoundMesh>(meshes));  // NOLINT

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::CompoundMesh()));

  meshes.push_back(sub_geom);
  EXPECT_FALSE(tesseract_geometry::isIdentical(*geom, tesseract_geometry::CompoundMesh(meshes)));
}

TEST(TesseractGeometryUnit, SDFMesh)  // NOLINT
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  vertices->emplace_back(1, 1, 0);
  vertices->emplace_back(1, -1, 0);
  vertices->emplace_back(-1, -1, 0);
  vertices->emplace_back(1, -1, 0);

  auto faces = std::make_shared<Eigen::VectorXi>();
  faces->resize(8);
  (*faces)(0) = 3;
  (*faces)(1) = 0;
  (*faces)(2) = 1;
  (*faces)(3) = 2;

  (*faces)(4) = 3;
  (*faces)(5) = 0;
  (*faces)(6) = 2;
  (*faces)(7) = 3;

  using T = tesseract_geometry::SDFMesh;
  auto geom = std::make_shared<T>(vertices, faces);
  EXPECT_TRUE(geom->getVertices() != nullptr);
  EXPECT_TRUE(geom->getFaces() != nullptr);
  EXPECT_TRUE(geom->getVertexCount() == 4);
  EXPECT_TRUE(geom->getFaceCount() == 2);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::SDF_MESH);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertices() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaces() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getVertexCount() == 4);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getFaceCount() == 2);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::SDF_MESH);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
  EXPECT_FALSE(
      tesseract_geometry::isIdentical(*geom,
                                      tesseract_geometry::SDFMesh(std::make_shared<tesseract_common::VectorVector3d>(),
                                                                  std::make_shared<Eigen::VectorXi>())));
}

TEST(TesseractGeometryUnit, Octree)  // NOLINT
{
  using T = tesseract_geometry::Octree;

  tesseract_geometry::PointCloud pc;
  auto octree = tesseract_geometry::createOctree(pc, 0.01, false);
  auto geom = std::make_shared<T>(std::move(octree), tesseract_geometry::OctreeSubType::BOX, false);
  EXPECT_TRUE(geom->getOctree() != nullptr);
  EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::BOX);
  EXPECT_EQ(geom->getType(), tesseract_geometry::GeometryType::OCTREE);
  EXPECT_FALSE(geom->getUUID().is_nil());

  auto geom_clone = geom->clone();
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getOctree() != nullptr);
  EXPECT_TRUE(std::static_pointer_cast<T>(geom_clone)->getSubType() == tesseract_geometry::OctreeSubType::BOX);
  EXPECT_EQ(geom_clone->getType(), tesseract_geometry::GeometryType::OCTREE);
  EXPECT_FALSE(geom_clone->getUUID().is_nil());
  EXPECT_NE(geom_clone->getUUID(), geom->getUUID());

  geom->setUUID(geom_clone->getUUID());
  EXPECT_EQ(geom_clone->getUUID(), geom->getUUID());

  // Test isIdentical
  EXPECT_TRUE(tesseract_geometry::isIdentical(*geom, *geom_clone));
}

TEST(TesseractGeometryUnit, LoadMeshUnit)  // NOLINT
{
  using namespace tesseract_geometry;

  tesseract_common::GeneralResourceLocator locator;
  std::string mesh_file = locator.locateResource("package://tesseract_support/meshes/sphere_p25m.stl")->getFilePath();
  std::vector<Mesh::Ptr> meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/sphere_p25m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/sphere_p25m.dae")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file);
  EXPECT_TRUE(meshes.size() == 2);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 42);
  EXPECT_TRUE(meshes[1]->getFaceCount() == 80);
  EXPECT_TRUE(meshes[1]->getVertexCount() == 42);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/sphere_p25m.dae")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 2 * 80);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 2 * 42);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/box_2m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 12);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 8);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/box_2m.ply")->getFilePath();
  meshes = createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true);
  EXPECT_TRUE(meshes.size() == 1);
  EXPECT_TRUE(meshes[0]->getFaceCount() == 12);
  EXPECT_TRUE(meshes[0]->getVertexCount() == 8);

  mesh_file = locator.locateResource("package://tesseract_support/meshes/box_2m.ply")->getFilePath();
  std::vector<ConvexMesh::Ptr> convex_meshes =
      createMeshFromPath<ConvexMesh>(mesh_file, Eigen::Vector3d(1, 1, 1), false, false);
  EXPECT_TRUE(convex_meshes.size() == 1);
  EXPECT_TRUE(convex_meshes[0]->getFaceCount() == 6);
  EXPECT_TRUE(convex_meshes[0]->getVertexCount() == 8);
}

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL

TEST(TesseractGeometryUnit, LoadMeshWithMaterialGltf2Unit)  // NOLINT
{
  using namespace tesseract_geometry;

  tesseract_common::GeneralResourceLocator locator;
  std::string mesh_file =
      locator.locateResource("package://tesseract_support/meshes/tesseract_material_mesh.glb")->getFilePath();
  std::vector<Mesh::Ptr> meshes =
      createMeshFromPath<Mesh>(mesh_file, Eigen::Vector3d(1, 1, 1), true, true, true, true, true);
  ASSERT_TRUE(meshes.size() == 4);

  auto& mesh0 = meshes[0];
  auto& mesh1 = meshes[1];
  auto& mesh2 = meshes[2];
  auto& mesh3 = meshes[3];

  EXPECT_EQ(mesh0->getFaceCount(), 34);
  EXPECT_EQ(mesh0->getVertexCount(), 68);
  EXPECT_EQ(mesh1->getFaceCount(), 15);
  EXPECT_EQ(mesh1->getVertexCount(), 17);
  EXPECT_EQ(mesh2->getFaceCount(), 15);
  EXPECT_EQ(mesh2->getVertexCount(), 17);
  EXPECT_EQ(mesh3->getFaceCount(), 2);
  EXPECT_EQ(mesh3->getVertexCount(), 4);

  auto mesh0_normals = mesh0->getNormals();
  ASSERT_TRUE(mesh0_normals != nullptr);
  EXPECT_EQ(mesh0_normals->size(), 68);
  auto mesh1_normals = mesh1->getNormals();
  ASSERT_TRUE(mesh1_normals != nullptr);
  EXPECT_EQ(mesh1_normals->size(), 17);
  auto mesh2_normals = mesh2->getNormals();
  ASSERT_TRUE(mesh2_normals != nullptr);
  EXPECT_EQ(mesh2_normals->size(), 17);
  auto mesh3_normals = mesh3->getNormals();
  ASSERT_TRUE(mesh3_normals != nullptr);
  EXPECT_EQ(mesh3_normals->size(), 4);

  auto mesh0_material = mesh0->getMaterial();
  EXPECT_TRUE(mesh0_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.7, 0.7, 0.7, 1), 0.01));
  EXPECT_NEAR(mesh0_material->getMetallicFactor(), 0.0, 0.01);
  EXPECT_NEAR(mesh0_material->getRoughnessFactor(), 0.5, 0.01);
  EXPECT_TRUE(mesh0_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  auto mesh1_material = mesh1->getMaterial();
  EXPECT_TRUE(mesh1_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.8, 0, 0, 1), 0.01));
  EXPECT_NEAR(mesh1_material->getMetallicFactor(), 0.8, 0.01);
  EXPECT_NEAR(mesh1_material->getRoughnessFactor(), 0.1, 0.01);
  EXPECT_TRUE(mesh1_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  auto mesh2_material = mesh2->getMaterial();
  EXPECT_TRUE(mesh2_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.05, 0.8, 0.05, 1), 0.01));
  EXPECT_NEAR(mesh2_material->getMetallicFactor(), 0.9, 0.01);
  EXPECT_NEAR(mesh2_material->getRoughnessFactor(), 0.7, 0.01);
  EXPECT_TRUE(mesh2_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0.1, 0.1, 0.5, 1), 0.01));

  auto mesh3_material = mesh3->getMaterial();
  EXPECT_TRUE(mesh3_material->getBaseColorFactor().isApprox(Eigen::Vector4d(1, 1, 1, 1), 0.01));
  EXPECT_NEAR(mesh3_material->getMetallicFactor(), 0, 0.01);
  EXPECT_NEAR(mesh3_material->getRoughnessFactor(), 0.5, 0.01);
  EXPECT_TRUE(mesh3_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

  EXPECT_TRUE(mesh0->getTextures() == nullptr);
  EXPECT_TRUE(mesh1->getTextures() == nullptr);
  EXPECT_TRUE(mesh2->getTextures() == nullptr);

  ASSERT_TRUE(mesh3->getTextures() != nullptr);
  ASSERT_EQ(mesh3->getTextures()->size(), 1);

  auto texture = mesh3->getTextures()->at(0);
  EXPECT_EQ(texture->getTextureImage()->getResourceContents().size(), 38212);
  EXPECT_EQ(texture->getUVs()->size(), 4);
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
