/**
 * @file link.cpp
 * @brief Parse link from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/collision.h>
#include <tesseract_urdf/inertial.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/visual.h>

namespace tesseract_urdf
{
tesseract_scene_graph::Link::Ptr
parseLink(const tinyxml2::XMLElement* xml_element,
          const tesseract_common::ResourceLocator& locator,
          bool make_convex_meshes,
          std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials)
{
  std::string link_name;
  if (tesseract_common::QueryStringAttribute(xml_element, "name", link_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Link: Missing or failed parsing attribute 'name'!"));

  auto l = std::make_shared<tesseract_scene_graph::Link>(link_name);

  // get inertia if it exists
  const tinyxml2::XMLElement* inertial = xml_element->FirstChildElement("inertial");
  if (inertial != nullptr)
  {
    try
    {
      l->inertial = parseInertial(inertial);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Link: Error parsing 'inertial' element for link '" + link_name + "'!"));
    }
  }

  // get visual if it exists
  for (const tinyxml2::XMLElement* visual = xml_element->FirstChildElement("visual"); visual != nullptr;
       visual = visual->NextSiblingElement("visual"))
  {
    tesseract_scene_graph::Visual::Ptr temp_visual;
    try
    {
      temp_visual = parseVisual(visual, locator, available_materials);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Link: Error parsing 'visual' element for link '" + link_name + "'!"));
    }

    l->visual.push_back(temp_visual);
  }

  // get collision if exists
  for (const tinyxml2::XMLElement* collision = xml_element->FirstChildElement("collision"); collision != nullptr;
       collision = collision->NextSiblingElement("collision"))
  {
    tesseract_scene_graph::Collision::Ptr temp_collision;
    try
    {
      temp_collision = parseCollision(collision, locator, make_convex_meshes);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Link: Error parsing 'collision' element for link '" + link_name + "'!"));
    }

    l->collision.push_back(temp_collision);
  }

  return l;
}

tinyxml2::XMLElement* writeLink(const std::shared_ptr<const tesseract_scene_graph::Link>& link,
                                tinyxml2::XMLDocument& doc,
                                const std::string& package_path)
{
  if (link == nullptr)
    std::throw_with_nested(std::runtime_error("Link is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(LINK_ELEMENT_NAME.data());

  // Set name
  xml_element->SetAttribute("name", link->getName().c_str());

  // Set inertia if it exists
  if (link->inertial != nullptr)
  {
    tinyxml2::XMLElement* xml_inertial = writeInertial(link->inertial, doc);
    xml_element->InsertEndChild(xml_inertial);
  }

  // Set visual if it exists
  int id = -1;
  if (link->visual.size() > 1)
    id = 0;
  for (const tesseract_scene_graph::Visual::Ptr& vis : link->visual)
  {
    try
    {
      std::filesystem::create_directory(std::filesystem::path(trailingSlash(package_path) + "visual/"));
      tinyxml2::XMLElement* xml_visual = writeVisual(vis, doc, package_path, link->getName(), id++);
      xml_element->InsertEndChild(xml_visual);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write visual to XML for link `" + link->getName() + "`!"));
    }
  }

  // Set collision if it exists
  id = -1;
  if (link->collision.size() > 1)
    id = 0;
  for (const tesseract_scene_graph::Collision::Ptr& col : link->collision)
  {
    try
    {
      std::filesystem::create_directory(std::filesystem::path(trailingSlash(package_path) + "collision/"));
      tinyxml2::XMLElement* xml_collision = writeCollision(col, doc, package_path, link->getName(), id++);
      xml_element->InsertEndChild(xml_collision);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Could not write collision to XML for link `" + link->getName() + "`!"));
    }
  }

  return xml_element;
}

}  // namespace tesseract_urdf
