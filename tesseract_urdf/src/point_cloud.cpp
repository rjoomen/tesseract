/**
 * @file point_cloud.cpp
 * @brief Parse PCL point cloud to octree from xml string
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

#include <console_bridge/console.h>
#include <pcl/io/pcd_io.h>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
#include <octomap/OcTree.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_geometry/impl/octree_utils.h>
#include <tesseract_urdf/point_cloud.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_urdf
{
tesseract_geometry::Octree::Ptr parsePointCloud(const tinyxml2::XMLElement* xml_element,
                                                const tesseract_common::ResourceLocator& locator,
                                                tesseract_geometry::OctreeSubType shape_type,
                                                bool prune)
{
  std::string filename;
  if (tesseract_common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("PointCloud: Missing or failed parsing attribute 'filename'!"));

  double resolution{ 0 };
  if (xml_element->QueryDoubleAttribute("resolution", &resolution) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("PointCloud: Missing or failed parsing point_cloud attribute "
                                              "'resolution'!"));

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tesseract_common::Resource::Ptr located_resource = locator.locateResource(filename);
  if (!located_resource || !located_resource->isFile() || !std::filesystem::exists(located_resource->getFilePath()))
  {
    // TODO: Handle point clouds that are not files
    CONSOLE_BRIDGE_logError("Point clouds can only be loaded from file");
    std::throw_with_nested(std::runtime_error("PointCloud: Unable to locate resource '" + filename + "'!"));
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(located_resource->getFilePath(), *cloud) == -1)
    std::throw_with_nested(std::runtime_error("PointCloud: Failed to import point cloud from '" + filename + "'!"));

  if (cloud->points.empty())
    std::throw_with_nested(std::runtime_error("PointCloud: Imported point cloud from '" + filename + "' is empty!"));

  auto octree = tesseract_geometry::createOctree(*cloud, resolution, prune);
  auto geom = std::make_shared<tesseract_geometry::Octree>(std::move(octree), shape_type, prune);
  if (geom == nullptr)
    std::throw_with_nested(std::runtime_error("PointCloud: Failed to create Tesseract Octree Geometry from point "
                                              "cloud!"));

  return geom;
}

}  // namespace tesseract_urdf
