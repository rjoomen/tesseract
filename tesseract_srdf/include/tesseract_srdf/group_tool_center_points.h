/**
 * @file group_tool_center_points.h
 * @brief Parse group tool center points data from srdf file
 *
 * @author Levi Armstrong
 * @date March 13, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#ifndef TESSERACT_SRDF_TOOL_CENTER_POINTS_H
#define TESSERACT_SRDF_TOOL_CENTER_POINTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/fwd.h>
#include <tesseract_srdf/kinematics_information.h>

namespace tinyxml2
{
class XMLElement;  // NOLINT
}

namespace tesseract_srdf
{
/**
 * @brief Parse groups tool center points from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Group Tool Center Points
 */
GroupTCPs parseGroupTCPs(const tesseract_scene_graph::SceneGraph& scene_graph,
                         const tinyxml2::XMLElement* srdf_xml,
                         const std::array<int, 3>& version);

}  // namespace tesseract_srdf

#endif  // TESSERACT_SRDF_TOOL_CENTER_POINTS_H
