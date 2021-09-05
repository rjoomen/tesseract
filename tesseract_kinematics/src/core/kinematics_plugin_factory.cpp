/**
 * @file kinematics_factory.h
 * @brief Kinematics Factory
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_loader.hpp>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

const std::string TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV = "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES";
const std::string TESSERACT_KINEMATICS_PLUGINS_ENV = "TESSERACT_KINEMATICS_PLUGINS";
const std::string CONFIG_KEY = "kinematic_plugins";
const std::string SEARCH_PATHS_KEY = "search_paths";
const std::string SEARCH_LIBRARIES_KEY = "search_libraries";
const std::string FWD_KIN_PLUGINS_KEY = "fwd_kin_plugins";
const std::string INV_KIN_PLUGINS_KEY = "inv_kin_plugins";

namespace tesseract_kinematics
{
KinematicsPluginFactory::KinematicsPluginFactory() : plugin_loader_(std::make_unique<tesseract_common::PluginLoader>())
{
  plugin_loader_->plugins_env = TESSERACT_KINEMATICS_PLUGINS_ENV;
  plugin_loader_->search_paths_env = TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES_ENV;
  plugin_loader_->search_paths.insert(TESSERACT_KINEMATICS_PLUGIN_PATH);
  boost::split(plugin_loader_->plugins, TESSERACT_KINEMATICS_PLUGINS, boost::is_any_of(":"), boost::token_compress_on);
}

KinematicsPluginFactory::KinematicsPluginFactory(YAML::Node config) : KinematicsPluginFactory()
{
  if (const YAML::Node& plugin_info = config[CONFIG_KEY])
  {
    if (const YAML::Node& search_paths = plugin_info[SEARCH_PATHS_KEY])
    {
      std::set<std::string> sp;
      try
      {
        sp = search_paths.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        CONSOLE_BRIDGE_logError("KinematicsPluginFactory: Constructor failed to cast '%s' to std::set<std::string>! "
                                "Details: %s",
                                SEARCH_PATHS_KEY.c_str(),
                                e.what());
      }
      plugin_loader_->search_paths.insert(sp.begin(), sp.end());
    }

    if (const YAML::Node& search_libraries = plugin_info[SEARCH_LIBRARIES_KEY])
    {
      std::set<std::string> sl;
      try
      {
        sl = search_libraries.as<std::set<std::string>>();
      }
      catch (const std::exception& e)
      {
        CONSOLE_BRIDGE_logError("KinematicsPluginFactory: Constructor failed to cast '%s' to std::set<std::string>! "
                                "Details: %s",
                                SEARCH_LIBRARIES_KEY.c_str(),
                                e.what());
      }
      plugin_loader_->plugins.insert(sl.begin(), sl.end());
    }

    if (const YAML::Node& fwd_kin_plugins = plugin_info[FWD_KIN_PLUGINS_KEY])
    {
      if (!fwd_kin_plugins.IsMap())
        throw std::runtime_error(FWD_KIN_PLUGINS_KEY + ", should contain a map of group names to solver plugins!");

      try
      {
        fwd_plugin_info_ = fwd_kin_plugins.as<std::map<std::string, tesseract_common::PluginInfoMap>>();
      }
      catch (const std::exception& e)
      {
        CONSOLE_BRIDGE_logError("KinematicsPluginFactory: Constructor failed to cast '%s' to std::map<std::string, "
                                "tesseract_common::PluginInfoMap>! Details: %s",
                                FWD_KIN_PLUGINS_KEY.c_str(),
                                e.what());
      }
    }

    if (const YAML::Node& inv_kin_plugins = plugin_info[INV_KIN_PLUGINS_KEY])
    {
      if (!inv_kin_plugins.IsMap())
        throw std::runtime_error(INV_KIN_PLUGINS_KEY + ", should contain a map of group names to solver plugins!");

      try
      {
        inv_plugin_info_ = inv_kin_plugins.as<std::map<std::string, tesseract_common::PluginInfoMap>>();
      }
      catch (const std::exception& e)
      {
        CONSOLE_BRIDGE_logError("KinematicsPluginFactory: Constructor failed to cast '%s' to std::map<std::string, "
                                "tesseract_common::PluginInfoMap>! Details: %s",
                                INV_KIN_PLUGINS_KEY.c_str(),
                                e.what());
      }
    }
  }
}

KinematicsPluginFactory::KinematicsPluginFactory(boost::filesystem::path config)
  : KinematicsPluginFactory(YAML::LoadFile(config.string()))
{
}

KinematicsPluginFactory::KinematicsPluginFactory(std::string config) : KinematicsPluginFactory(YAML::Load(config)) {}

// This prevents it from being defined inline.
// If not the forward declare of PluginLoader cause compiler error.
KinematicsPluginFactory::~KinematicsPluginFactory() = default;

void KinematicsPluginFactory::addSearchPath(const std::string& path) { plugin_loader_->search_paths.insert(path); }

const std::set<std::string>& KinematicsPluginFactory::getSearchPaths() const { return plugin_loader_->search_paths; }

void KinematicsPluginFactory::addSearchLibrary(const std::string& library_name)
{
  plugin_loader_->plugins.insert(library_name);
}

const std::set<std::string>& KinematicsPluginFactory::getSearchLibraries() const { return plugin_loader_->plugins; }

void KinematicsPluginFactory::addFwdKinPlugin(const std::string& group_name,
                                              const std::string& solver_name,
                                              tesseract_common::PluginInfo plugin_info)
{
  fwd_plugin_info_[group_name][solver_name] = plugin_info;
}

void KinematicsPluginFactory::removeFwdKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed fwd kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.erase(solver_it);
  if (group_it->second.empty())
    fwd_plugin_info_.erase(group_it);
}

void KinematicsPluginFactory::setDefaultFwdKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default fwd kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default fwd kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  for (auto& solver : group_it->second)
    solver.second.is_default = false;

  solver_it->second.is_default = true;
}

std::string KinematicsPluginFactory::getDefaultFwdKinPlugin(const std::string& group_name)
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to get default fwd kin solver for a group '" + group_name +
                             "' that does not exist!");

  for (auto& solver : group_it->second)
  {
    if (solver.second.is_default)
      return solver.first;
  }

  // If one is not explicitly set as the default use the first one.
  return group_it->second.begin()->first;
}

void KinematicsPluginFactory::addInvKinPlugin(const std::string& group_name,
                                              const std::string& solver_name,
                                              tesseract_common::PluginInfo plugin_info)
{
  inv_plugin_info_[group_name][solver_name] = plugin_info;
}

void KinematicsPluginFactory::removeInvKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to removed inv kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  group_it->second.erase(solver_it);
  if (group_it->second.empty())
    fwd_plugin_info_.erase(group_it);
}

void KinematicsPluginFactory::setDefaultInvKinPlugin(const std::string& group_name, const std::string& solver_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default inv kin solver '" + solver_name +
                             "' for a group '" + group_name + "' that does not exist!");

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to set default inv kin solver '" + solver_name +
                             "' that does not exist for group '" + group_name + "'!");

  for (auto& solver : group_it->second)
    solver.second.is_default = false;

  solver_it->second.is_default = true;
}

std::string KinematicsPluginFactory::getDefaultInvKinPlugin(const std::string& group_name)
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
    throw std::runtime_error("KinematicsPluginFactory, tried to get default inv kin solver for a group '" + group_name +
                             "' that does not exist!");

  for (auto& solver : group_it->second)
  {
    if (solver.second.is_default)
      return solver.first;
  }

  // If one is not explicitly set as the default use the first one.
  return group_it->second.begin()->first;
}

ForwardKinematics::UPtr
KinematicsPluginFactory::createFwdKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  auto group_it = fwd_plugin_info_.find(group_name);
  if (group_it == fwd_plugin_info_.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get fwd kin solver '%s' for a group '%s' that does not "
                           "exist!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get fwd kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return createFwdKin(group_name, solver_name, solver_it->second, scene_graph, scene_state);
}

ForwardKinematics::UPtr
KinematicsPluginFactory::createFwdKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_common::PluginInfo& plugin_info,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = fwd_kin_factories_.find(plugin_info.class_name);
    if (it != fwd_kin_factories_.end())
      return it->second->create(group_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_->instantiate<FwdKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    fwd_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(group_name, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

InverseKinematics::UPtr
KinematicsPluginFactory::createInvKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  auto group_it = inv_plugin_info_.find(group_name);
  if (group_it == inv_plugin_info_.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get inv kin solver '%s' for a group '%s' that does not "
                           "exist!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  auto solver_it = group_it->second.find(solver_name);
  if (solver_it == group_it->second.end())
  {
    CONSOLE_BRIDGE_logWarn("KinematicsPluginFactory, tried to get inv kin solver '%s' that does not exist for group "
                           "'%s'!",
                           solver_name.c_str(),
                           group_name.c_str());
    return nullptr;
  }

  return createInvKin(group_name, solver_name, solver_it->second, scene_graph, scene_state);
}

InverseKinematics::UPtr
KinematicsPluginFactory::createInvKin(const std::string& group_name,
                                      const std::string& solver_name,
                                      const tesseract_common::PluginInfo& plugin_info,
                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_scene_graph::SceneState& scene_state) const
{
  try
  {
    auto it = inv_kin_factories_.find(plugin_info.class_name);
    if (it != inv_kin_factories_.end())
      return it->second->create(group_name, scene_graph, scene_state, *this, plugin_info.config);

    auto plugin = plugin_loader_->instantiate<InvKinFactory>(plugin_info.class_name);
    if (plugin == nullptr)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
      return nullptr;
    }
    inv_kin_factories_[plugin_info.class_name] = plugin;
    return plugin->create(group_name, scene_graph, scene_state, *this, plugin_info.config);
  }
  catch (const std::exception&)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load symbol '%s'", plugin_info.class_name.c_str());
    return nullptr;
  }
}

void KinematicsPluginFactory::saveConfig(tesseract_common::fs::path file_path) const
{
  YAML::Node config = getConfig();
  std::ofstream fout(file_path.string());
  fout << config;
}

YAML::Node KinematicsPluginFactory::getConfig() const
{
  YAML::Node config, kinematic_plugins;
  if (!plugin_loader_->search_paths.empty())
    kinematic_plugins[SEARCH_PATHS_KEY] = plugin_loader_->search_paths;

  if (!plugin_loader_->plugins.empty())
    kinematic_plugins[SEARCH_LIBRARIES_KEY] = plugin_loader_->plugins;

  if (!fwd_plugin_info_.empty())
    kinematic_plugins[FWD_KIN_PLUGINS_KEY] = fwd_plugin_info_;

  if (!inv_plugin_info_.empty())
    kinematic_plugins[INV_KIN_PLUGINS_KEY] = inv_plugin_info_;

  config[CONFIG_KEY] = kinematic_plugins;

  return config;
}

}  // namespace tesseract_kinematics