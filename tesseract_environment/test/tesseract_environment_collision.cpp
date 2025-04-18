#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands/add_link_command.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>

#include <tesseract_srdf/srdf_model.h>

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_collision;
using namespace tesseract_environment;

SceneGraph::Ptr getSceneGraph(const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf";
  return tesseract_urdf::parseURDFFile(locator.locateResource(path)->getFilePath(), locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph, const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf";

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(path)->getFilePath(), locator);

  return srdf;
}

tesseract_environment::Environment::UPtr getEnvironment()
{
  tesseract_common::GeneralResourceLocator locator;

  auto scene_graph = getSceneGraph(locator);
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph, locator);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_unique<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  Link link_1("link_n1");
  {
    Visual::Ptr v = std::make_shared<Visual>();
    v->origin.translation() = Eigen::Vector3d(0, 0, 0);
    v->material = std::make_shared<Material>("test_material");
    v->material->color = Eigen::Vector4d(1, 0, 0, 1);
    v->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
    v->name = "link1_visual";
    link_1.visual.push_back(v);

    Collision::Ptr c = std::make_shared<Collision>();
    c->origin.translation() = v->origin.translation();
    c->geometry = v->geometry;
    c->name = "link1_collision";
    link_1.collision.push_back(c);
  }

  Link link_2("link_n2");
  {
    Visual::Ptr v = std::make_shared<Visual>();
    v->origin.translation() = Eigen::Vector3d(1.5, 0, 0);
    v->material = std::make_shared<Material>("test_material");
    v->material->color = Eigen::Vector4d(1, 1, 1, 1);
    v->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
    v->name = "link2_visual";
    link_2.visual.push_back(v);

    Collision::Ptr c = std::make_shared<Collision>();
    c->origin.translation() = v->origin.translation();
    c->geometry = v->geometry;
    c->name = "link2_collision";
    link_2.collision.push_back(c);
  }

  auto cmd1 = std::make_shared<AddLinkCommand>(link_1, true);
  env->applyCommand(cmd1);

  auto cmd2 = std::make_shared<AddLinkCommand>(link_2, true);
  env->applyCommand(cmd2);

  return env;
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentDiscreteCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();

  // Setup collision margin data
  CollisionCheckConfig collision_check_config;
  collision_check_config.longest_valid_segment_length = 0.1;
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;

  ContactManagerConfig contact_manager_config(0.0);

  {  // Setup collision checker
    DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
    {  // Check for collisions
      tesseract_collision::ContactResultMap collision;
      std::vector<std::string> active_links = { "link_n1" };
      manager->setActiveCollisionObjects(active_links);
      manager->contactTest(collision, collision_check_config.contact_request);
      EXPECT_FALSE(collision.empty());
    }

    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0, 0, 1.5);
    manager->setCollisionObjectsTransform("link_n1", pose);
    manager->applyContactManagerConfig(contact_manager_config);

    // Check for collisions
    tesseract_collision::ContactResultMap collision;
    manager->contactTest(collision, collision_check_config.contact_request);

    EXPECT_FALSE(collision.empty());
  }

  env->setActiveDiscreteContactManager("BulletDiscreteBVHManager");

  {  // Setup collision checker
    DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
    {  // Check for collisions
      tesseract_collision::ContactResultMap collision;
      std::vector<std::string> active_links = { "link_n1" };
      manager->setActiveCollisionObjects(active_links);
      manager->contactTest(collision, collision_check_config.contact_request);
      EXPECT_FALSE(collision.empty());
    }

    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0, 0, 1.5);
    manager->setCollisionObjectsTransform("link_n1", pose);
    manager->applyContactManagerConfig(contact_manager_config);

    // Check for collisions
    tesseract_collision::ContactResultMap collision;
    manager->contactTest(collision, collision_check_config.contact_request);

    EXPECT_FALSE(collision.empty());
  }
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentContinuousCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();

  // Setup collision margin data
  CollisionCheckConfig collision_check_config;
  collision_check_config.longest_valid_segment_length = 0.1;
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;

  ContactManagerConfig contact_manager_config(0.0);

  // Setup collision checker
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, collision_check_config.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(0, -2, 1.5);
  auto pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0, 2, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose1, pose2);
  manager->applyContactManagerConfig(contact_manager_config);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, collision_check_config.contact_request);

  EXPECT_FALSE(collision.empty());
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentClearDiscreteCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();
  env->clearCachedDiscreteContactManager();

  // Setup collision margin data
  CollisionCheckConfig collision_check_config;
  collision_check_config.longest_valid_segment_length = 0.1;
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;

  ContactManagerConfig contact_manager_config(0.0);

  // Setup collision checker
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, collision_check_config.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0, 0, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose);
  manager->applyContactManagerConfig(contact_manager_config);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, collision_check_config.contact_request);

  EXPECT_FALSE(collision.empty());
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentClearContinuousCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();
  env->clearCachedContinuousContactManager();

  // Setup collision margin data
  CollisionCheckConfig collision_check_config;
  collision_check_config.longest_valid_segment_length = 0.1;
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;

  ContactManagerConfig contact_manager_config(0.0);

  // Setup collision checker
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, collision_check_config.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(0, -2, 1.5);
  auto pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0, 2, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose1, pose2);
  manager->applyContactManagerConfig(contact_manager_config);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, collision_check_config.contact_request);

  EXPECT_FALSE(collision.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
