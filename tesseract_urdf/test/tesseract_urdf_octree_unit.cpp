#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract_common/utils.h>
#include <octomap/OcTree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_urdf/octree.h>
#include <tesseract_urdf/octomap.h>
#include <tesseract_common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_octree)  // NOLINT
{
  tesseract_common::GeneralResourceLocator resource_locator;
  //  {
  //    // Create octomap and add save it
  //    pcl::PointCloud<pcl::PointXYZ> full_cloud;
  //    double delta = 0.05;
  //    int length = static_cast<int>(1 / delta);

  //    for (int x = 0; x < length; ++x)
  //      for (int y = 0; y < length; ++y)
  //        for (int z = 0; z < length; ++z)
  //          full_cloud.push_back(pcl::PointXYZ(-0.5f + static_cast<float>(x * delta),
  //                                             -0.5f + static_cast<float>(y * delta),
  //                                             -0.5f + static_cast<float>(z * delta)));

  //    pcl::io::savePCDFile(tesseract_common::getTempPath() + "box_pcd.pcd", full_cloud, true);
  //  }

  {
    std::string str = R"(<tesseract:octomap shape_type="box" extra="0 0 0">
                           <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt" extra="0 0 0"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="box" prune="true">
                           <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="sphere_inside" prune="true">
                           <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::SPHERE_INSIDE);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

#ifdef TESSERACT_PARSE_POINT_CLOUDS
  {
    std::string str = R"(<tesseract:octomap shape_type="box">
                           <tesseract:point_cloud filename="package://tesseract_support/meshes/box_pcd.pcd" resolution="0.1"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 1000);
    EXPECT_NEAR(geom->getOctree()->getResolution(), 0.1, 1e-5);
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="box" prune="true">
                           <tesseract:point_cloud filename="package://tesseract_support/meshes/box_pcd.pcd" resolution="0.1"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 496);
    EXPECT_NEAR(geom->getOctree()->getResolution(), 0.1, 1e-5);
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="box" prune="true">
                           <tesseract:point_cloud />
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {  // Failure invalid resolution
    std::string str = R"(<tesseract:octomap shape_type="box">
                           <tesseract:point_cloud filename="package://tesseract_support/meshes/box_pcd.pcd" resolution="a"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {  // Failure resource does not exist
    std::string str = R"(<tesseract:octomap shape_type="box">
                           <tesseract:point_cloud filename="package://tesseract_support/meshes/does_not_exist.pcd" resolution="0.1"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {  // Failure resource is not a point cloud
    std::string str = R"(<tesseract:octomap shape_type="box">
                           <tesseract:point_cloud filename="package://tesseract_support/meshes/box_2m.bt" resolution="0.1"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }
#endif

  {
    std::string str = R"(<tesseract:octomap shape_type="sphere_outside" prune="true">
                           <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_TRUE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
    EXPECT_TRUE(geom->getSubType() == tesseract_geometry::OctreeSubType::SPHERE_OUTSIDE);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="sphere_outside" prune="true">
                           <tesseract:octree />
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="sphere_outside" prune="true">
                           <tesseract:octree filename="abc"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="sphere_outside" prune="true">
                           <tesseract:octree />
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="star" prune="true">
                           <tesseract:octree filename="package://tesseract_support/meshes/box_2m.bt"/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {
    std::string str = R"(<tesseract:octomap shape_type="star" prune="true">
                           <tesseract:octree/>
                         </tesseract:octomap>)";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }

  {
    std::string str = "<tesseract:octomap />";
    tesseract_geometry::Octree::Ptr geom;
    EXPECT_FALSE(runTest<tesseract_geometry::Octree::Ptr>(
        geom, &tesseract_urdf::parseOctomap, str, tesseract_urdf::OCTOMAP_ELEMENT_NAME.data(), resource_locator, true));
  }
}

TEST(TesseractURDFUnit, write_octree)  // NOLINT
{
  {
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctree, text, tesseract_common::getTempPath(), std::string("oct0.bt")));
    EXPECT_NE(text, "");
  }

  {  // Trigger failed-to-write
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctree, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {  // trigger nullptr input
    tesseract_geometry::Octree::Ptr geom = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctree, text, tesseract_common::getTempPath(), std::string("oct2.bt")));
    EXPECT_EQ(text, "");
  }
}

TEST(TesseractURDFUnit, write_octomap)  // NOLINT
{
  {  // box
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctomap, text, tesseract_common::getTempPath(), std::string("octo0.bt")));
    EXPECT_NE(text, "");
  }

  {  // sphere inside
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::SPHERE_INSIDE);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctomap, text, tesseract_common::getTempPath(), std::string("octo1.bt")));
    EXPECT_NE(text, "");
  }

  {  // sphere outside
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::SPHERE_OUTSIDE);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctomap, text, tesseract_common::getTempPath(), std::string("octo2.bt")));
    EXPECT_NE(text, "");
  }

  {  // Trigger failed-to-write
    tesseract_geometry::Octree::Ptr geom = std::make_shared<tesseract_geometry::Octree>(
        std::make_shared<octomap::OcTree>(1.0), tesseract_geometry::OctreeSubType::BOX);
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctomap, text, tesseract_common::getTempPath(), std::string("")));
    EXPECT_EQ(text, "");
  }

  {  // trigger nullptr input
    tesseract_geometry::Octree::Ptr geom = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_geometry::Octree::Ptr>(
                  geom, &tesseract_urdf::writeOctomap, text, tesseract_common::getTempPath(), std::string("oct2.bt")));
    EXPECT_EQ(text, "");
  }
}
