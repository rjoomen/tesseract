#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/limits.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_limits)  // NOLINT
{
  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="4" extra="0 0 0"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit upper="2" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit lower="1" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = R"(<limit effort="3" velocity="4" acceleration="2"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
    EXPECT_NEAR(elem->acceleration, 2, 1e-8);
  }

  {
    std::string str = R"(<limit effort="3" velocity="4" jerk="2"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
    EXPECT_NEAR(elem->jerk, 2, 1e-8);
  }

  {
    std::string str = R"(<limit lower="a" upper="2" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit lower="1" upper="a" effort="3" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="a" velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="a"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="4" acceleration="a"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit lower="1" upper="2" effort="3" velocity="4" jerk="a"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit velocity="4"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit acceleration="2"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = R"(<limit effort="3"/>)";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }

  {
    std::string str = "<limit />";
    tesseract_scene_graph::JointLimits::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::JointLimits::Ptr>(
        elem, &tesseract_urdf::parseLimits, str, tesseract_urdf::LIMITS_ELEMENT_NAME.data()));
  }
}

TEST(TesseractURDFUnit, write_limits)  // NOLINT
{
  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_TRUE(text == R"(<limit effort="0" velocity="0"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    limits->lower = 1.0;
    limits->upper = 2.0;
    limits->effort = 3.0;
    limits->velocity = 4.0;
    limits->acceleration = 2.0;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, R"(<limit lower="1" upper="2" effort="3" velocity="4"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    limits->lower = 1.0;
    limits->upper = 2.0;
    limits->effort = 3.0;
    limits->velocity = 4.0;
    limits->acceleration = 3.0;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, R"(<limit lower="1" upper="2" effort="3" velocity="4" acceleration="3"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    limits->lower = 1.0;
    limits->upper = 2.0;
    limits->effort = 3.0;
    limits->velocity = 4.0;
    limits->jerk = 0.0;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, R"(<limit lower="1" upper="2" effort="3" velocity="4"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    limits->lower = 1.0;
    limits->upper = 2.0;
    limits->effort = 3.0;
    limits->velocity = 4.0;
    limits->jerk = 1000.0;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, R"(<limit lower="1" upper="2" effort="3" velocity="4"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    limits->lower = 1.0;
    limits->upper = 2.0;
    limits->effort = 3.0;
    limits->velocity = 4.0;
    limits->jerk = 500.0;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, R"(<limit lower="1" upper="2" effort="3" velocity="4" jerk="500"/>)");
  }

  {
    tesseract_scene_graph::JointLimits::Ptr limits = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::JointLimits::Ptr>(limits, &tesseract_urdf::writeLimits, text));
    EXPECT_EQ(text, "");
  }
}
