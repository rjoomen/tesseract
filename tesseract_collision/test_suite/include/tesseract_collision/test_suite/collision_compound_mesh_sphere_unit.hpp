#ifndef TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_SPHERE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_SPHERE_UNIT_HPP

#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_collision::test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  auto resource_locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto compound_mesh_resource = resource_locator->locateResource("package://tesseract_support/meshes/box_box.dae");
  auto meshes =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::ConvexMesh>(compound_mesh_resource->getFilePath());
  CollisionShapePtr box = std::make_shared<tesseract_geometry::CompoundMesh>(meshes);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("box_link");

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses);
  checker.disableCollisionObject("thin_box_link");

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere = std::make_shared<tesseract_geometry::Sphere>(0.25);

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj3_shapes, obj3_poses);

  /////////////////////////////////////////////
  // Add box and remove
  /////////////////////////////////////////////
  CollisionShapePtr remove_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d remove_box_pose;
  remove_box_pose.setIdentity();

  CollisionShapesConst obj4_shapes;
  tesseract_common::VectorIsometry3d obj4_poses;
  obj4_shapes.push_back(remove_box);
  obj4_poses.push_back(remove_box_pose);

  checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses);
  EXPECT_TRUE(checker.getCollisionObjects().size() == 4);
  EXPECT_TRUE(checker.hasCollisionObject("remove_box_link"));
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract_common::VectorIsometry3d()));

  /////////////////////////////////////////////
  // Check sizes
  /////////////////////////////////////////////
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
  for (const auto& co : checker.getCollisionObjects())
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co).size() == 1);
    for (const auto& cgt : checker.getCollisionObjectGeometriesTransforms(co))
    {
      EXPECT_TRUE(cgt.isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
  }
}

}  // namespace detail

inline void runTest(DiscreteContactManager& checker)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  // Call it again to test adding same object
  detail::addCollisionObjects(checker);

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  std::vector<std::string> active_links{ "box_link", "sphere_link" };
  checker.setActiveCollisionObjects(active_links);
  std::vector<std::string> check_active_links = checker.getActiveCollisionObjects();
  EXPECT_TRUE(tesseract_common::isIdentical<std::string>(active_links, check_active_links, false));

  EXPECT_TRUE(checker.getContactAllowedValidator() == nullptr);

  checker.setDefaultCollisionMargin(0.1);
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));

  ContactResultVector result_vector;
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.19053635, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.5)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (-0.05)) > 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][2], 0.0, 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.2, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0594636, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.2, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.25, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * -1.0, 0.001);

  // Compound mesh so check shape id
  EXPECT_EQ(result_vector[0].shape_id[static_cast<size_t>(idx[0])], 0);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(0, 0, -0.3);
  result.clear();
  result_vector.clear();

  // Use different method for setting transforms
  checker.setCollisionObjectsTransform("sphere_link", location["sphere_link"]);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenCopyResults(result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();

  checker.setCollisionMarginData(CollisionMarginData(0.251));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.251, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.1094636, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.5)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (0.75)) > 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][2], 0.0, 0.001);
  }
  else
  {
    // Increased tolernace because of FCL from 0.001 to 0.002
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.0, 0.002);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.0, 0.002);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0594636, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.0, 0.002);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.0, 0.002);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], -0.05, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * -1.0, 0.001);

  // Compound mesh so check shape id
  EXPECT_EQ(result_vector[0].shape_id[static_cast<size_t>(idx[0])], 0);

  /////////////////////////////////////////////
  // Test collision against second shape
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  location["sphere_link"].translation() = Eigen::Vector3d(0, 2.75, 0);

  checker.setCollisionObjectsTransform("sphere_link", location["sphere_link"]);
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  result.flattenMoveResults(result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.03130736, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  if (result_vector[0].single_contact_point)
  {
    EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
    EXPECT_FALSE(std::abs(result_vector[0].nearest_points[0][0] - (0.5)) > 0.001 &&
                 std::abs(result_vector[0].nearest_points[0][0] - (0.75)) > 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][1], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[0][2], 0.0, 0.001);
  }
  else
  {
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 2.4702682, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][2], 0.0297318, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.0, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 2.5014002, 0.001);
    EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][2], 0.0264228, 0.001);
  }

  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.9943989, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * -0.1056915, 0.001);

  // Compound mesh so check shape id
  EXPECT_EQ(result_vector[0].shape_id[static_cast<size_t>(idx[0])], 0);
}

}  // namespace tesseract_collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_COMPOUND_MESH_SPHERE_UNIT_HPP
