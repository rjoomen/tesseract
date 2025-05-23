/**
 * @file add_link_command.h
 * @brief Used to add link and joint to environment
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_ADD_LINK_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_LINK_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_scene_graph/fwd.h>

namespace boost::serialization
{
class access;
}

namespace tesseract_environment
{
class AddLinkCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddLinkCommand>;
  using ConstPtr = std::shared_ptr<const AddLinkCommand>;

  AddLinkCommand();
  /**
   * @brief Adds or replace a link to the environment
   *
   * If the link exists and replace_allowed equals true:
   *
   *        This command should replace the current link with the new link
   *
   * If the link exists and replace_allowed equals false:
   *
   *        This command should result in an error
   *
   * If the link does not exist:
   *
   *        This command should attach the link to the root link with a fixed joint
   *        with a joint name of joint_{link name}".
   *
   * @param link The link to be added to the graph
   * @param replace_allowed If true then if the link exists it will be replaced, otherwise if false it will fail.
   */
  AddLinkCommand(const tesseract_scene_graph::Link& link, bool replace_allowed = false);

  /**
   * @brief Adds a link and joint in the environment
   *
   * If the link and joint exist and replace is allowed
   *
   *        This command will replace both link and joint if the link is the child link, otherwise this results in error
   *
   * If the link and joint exist and replace is not allowed
   *
   *        This command should result in an error
   *
   * If the link or joint only exists:
   *
   *        This command should result in an error
   *
   * @param link The link to be added to the graph
   * @param joint The joint to be used to attach link to environment
   * @param replace_allowed If true then if the link and joint exists it will be replaced, otherwise if false it will
   * fail.
   */
  AddLinkCommand(const tesseract_scene_graph::Link& link,
                 const tesseract_scene_graph::Joint& joint,
                 bool replace_allowed = false);

  const std::shared_ptr<const tesseract_scene_graph::Link>& getLink() const;
  const std::shared_ptr<const tesseract_scene_graph::Joint>& getJoint() const;
  bool replaceAllowed() const;

  bool operator==(const AddLinkCommand& rhs) const;
  bool operator!=(const AddLinkCommand& rhs) const;

private:
  std::shared_ptr<const tesseract_scene_graph::Link> link_;
  std::shared_ptr<const tesseract_scene_graph::Joint> joint_;
  bool replace_allowed_{ false };

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

BOOST_CLASS_EXPORT_KEY(tesseract_environment::AddLinkCommand)

#endif  // TESSERACT_ENVIRONMENT_ADD_COMMAND_H
