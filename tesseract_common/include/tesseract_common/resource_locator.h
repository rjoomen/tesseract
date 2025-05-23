/**
 * @file resource_locator.h
 * @brief Locate and retrieve resource data
 *
 * @author John Wason
 * @date October 25, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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
#ifndef TESSERACT_COMMON_RESOURCE_LOCATOR_H
#define TESSERACT_COMMON_RESOURCE_LOCATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <filesystem>

namespace boost::serialization
{
class access;
}

namespace tesseract_common
{
// Forward declare
class Resource;
struct Serialization;

/** @brief Abstract class for resource loaders */
class ResourceLocator
{
public:
  using Ptr = std::shared_ptr<ResourceLocator>;
  using ConstPtr = std::shared_ptr<const ResourceLocator>;

  virtual ~ResourceLocator() = default;

  /**
   * @brief Locate a resource based on a URL
   *
   * @param url The URL of the resource
   * @return A shared pointer to a Resource instance, or nullptr if not found
   */
  virtual std::shared_ptr<Resource> locateResource(const std::string& url) const = 0;

  bool operator==(const ResourceLocator& rhs) const;
  bool operator!=(const ResourceLocator& rhs) const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief A general resource loaders using environment variable
 * @details Also can set this environment variable TESSERACT_RESOURCE_PATH
 * with ':' separated directories and then use the directires as package names
 */
class GeneralResourceLocator : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<GeneralResourceLocator>;
  using ConstPtr = std::shared_ptr<const GeneralResourceLocator>;
  /**
   * @brief Construct a new General Resource Locator object using the TESSERACT_RESOURCE_PATH environment variable
   *
   * @param environment_variables A vector of environment variables to search for paths
   */
  GeneralResourceLocator(const std::vector<std::string>& environment_variables = { "TESSERACT_RESOURCE_PATH",
                                                                                   "ROS_PACKAGE_PATH" });
  /**
   * @brief Construct a new General Resource Locator object using the provided paths and/or the TESSERACT_RESOURCE_PATH
   * environment variable
   *
   * @param paths A vector of paths to search for resources
   * @param environment_variables A vector of environment variables to search for paths
   */
  GeneralResourceLocator(const std::vector<std::filesystem::path>& paths,
                         const std::vector<std::string>& environment_variables = { "TESSERACT_RESOURCE_PATH",
                                                                                   "ROS_PACKAGE_PATH" });
  GeneralResourceLocator(const GeneralResourceLocator&) = default;
  GeneralResourceLocator& operator=(const GeneralResourceLocator&) = default;
  GeneralResourceLocator(GeneralResourceLocator&&) = default;
  GeneralResourceLocator& operator=(GeneralResourceLocator&&) = default;
  ~GeneralResourceLocator() override = default;

  std::shared_ptr<Resource> locateResource(const std::string& url) const override;

  bool operator==(const GeneralResourceLocator& rhs) const;
  bool operator!=(const GeneralResourceLocator& rhs) const;

  /**
   * @brief Add path to the resource locator
   *
   * @param path The path to add. Must be a directory
   * @return true Success
   * @return false Directory does not exist
   */
  bool addPath(const std::filesystem::path& path);

  /**
   * @brief Load paths from an environment variable
   *
   * @param environment_variable The environment variable to load paths from
   * @return true Success
   * @return false Environment variable does not exist
   */
  bool loadEnvironmentVariable(const std::string& environment_variable);

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::unordered_map<std::string, std::string> package_paths_;

  void processToken(const std::string& token);
};

/**  @brief Represents resource data available from a file or url */
class Resource : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  /**
   * @brief Returns true if the located resource is a local file
   *
   * @return true if the resource is a local file, otherwise false
   */
  virtual bool isFile() const = 0;

  /**
   * @brief Get the original URL used to locate the file
   *
   * @return The URL of the resource
   */
  virtual std::string getUrl() const = 0;

  /**
   * @brief Get the file path of the resource. Only valid if isFile() is true.
   *
   * @return The file path to the resource
   */
  virtual std::string getFilePath() const = 0;

  /**
   * @brief Get the resource as bytes. This function may block
   *
   * @return Resource bytes as a uint8_t vector
   */
  virtual std::vector<uint8_t> getResourceContents() const = 0;

  /**
   * @brief Get the resource as a std::istream. This function and the returned stream may block
   *
   * @return A std::istream shared pointer for the resource data
   */
  virtual std::shared_ptr<std::istream> getResourceContentStream() const = 0;

  bool operator==(const Resource& rhs) const;
  bool operator!=(const Resource& rhs) const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

using SimpleResourceLocatorFn = std::function<std::string(const std::string&)>;

/** @brief Resource implementation for a local file */
class SimpleLocatedResource : public Resource
{
public:
  using Ptr = std::shared_ptr<SimpleLocatedResource>;
  using ConstPtr = std::shared_ptr<const SimpleLocatedResource>;

  /** @brief This is for boost serialization do not use directly */
  SimpleLocatedResource() = default;

  /**
   * @brief A generic resource
   * @param url The url to the resource
   * @param filename The file path to the resource
   * @param parent The locator used to locate the resource
   */
  SimpleLocatedResource(std::string url, std::string filename, ResourceLocator::ConstPtr parent = nullptr);
  ~SimpleLocatedResource() override = default;
  SimpleLocatedResource(const SimpleLocatedResource&) = default;
  SimpleLocatedResource& operator=(const SimpleLocatedResource&) = default;
  SimpleLocatedResource(SimpleLocatedResource&&) = default;
  SimpleLocatedResource& operator=(SimpleLocatedResource&&) = default;

  bool isFile() const override final;

  std::string getUrl() const override final;

  std::string getFilePath() const override final;

  std::vector<uint8_t> getResourceContents() const override final;

  std::shared_ptr<std::istream> getResourceContentStream() const override final;

  Resource::Ptr locateResource(const std::string& url) const override final;

  bool operator==(const SimpleLocatedResource& rhs) const;
  bool operator!=(const SimpleLocatedResource& rhs) const;

private:
  std::string url_;
  std::string filename_;
  ResourceLocator::ConstPtr parent_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class BytesResource : public tesseract_common::Resource
{
public:
  /** @brief This is for boost serialization do not use directly */
  BytesResource() = default;

  BytesResource(std::string url, std::vector<uint8_t> bytes, ResourceLocator::ConstPtr parent = nullptr);
  BytesResource(std::string url, const uint8_t* bytes, size_t bytes_len, ResourceLocator::ConstPtr parent = nullptr);
  ~BytesResource() override = default;
  BytesResource(const BytesResource&) = default;
  BytesResource& operator=(const BytesResource&) = default;
  BytesResource(BytesResource&&) = default;
  BytesResource& operator=(BytesResource&&) = default;

  bool isFile() const override final;
  std::string getUrl() const override final;
  std::string getFilePath() const override final;
  std::vector<uint8_t> getResourceContents() const override final;
  std::shared_ptr<std::istream> getResourceContentStream() const override final;
  Resource::Ptr locateResource(const std::string& url) const override final;

  bool operator==(const BytesResource& rhs) const;
  bool operator!=(const BytesResource& rhs) const;

private:
  std::string url_;
  std::vector<uint8_t> bytes_;
  ResourceLocator::ConstPtr parent_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_common
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_common::ResourceLocator)
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_common::Resource)

BOOST_CLASS_EXPORT_KEY(tesseract_common::GeneralResourceLocator)
BOOST_CLASS_EXPORT_KEY(tesseract_common::SimpleLocatedResource)
BOOST_CLASS_EXPORT_KEY(tesseract_common::BytesResource)

#endif  // TESSERACT_COMMON_RESOURCE_LOCATOR_H
