/**
 * @file fcl_discrete_managers.cpp
 * @brief Tesseract ROS FCL contact checker implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_common/contact_allowed_validator.h>

namespace tesseract_collision::tesseract_collision_fcl
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

FCLDiscreteBVHManager::FCLDiscreteBVHManager(std::string name) : name_(std::move(name))
{
  static_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
  dynamic_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
  collision_margin_data_ = CollisionMarginData(0);
}

std::string FCLDiscreteBVHManager::getName() const { return name_; }

DiscreteContactManager::UPtr FCLDiscreteBVHManager::clone() const
{
  auto manager = std::make_unique<FCLDiscreteBVHManager>();

  for (const auto& cow : link2cow_)
    manager->addCollisionObject(cow.second->clone());

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(collision_margin_data_);
  manager->setContactAllowedValidator(validator_);

  return manager;
}

bool FCLDiscreteBVHManager::addCollisionObject(const std::string& name,
                                               const int& mask_id,
                                               const CollisionShapesConst& shapes,
                                               const tesseract_common::VectorIsometry3d& shape_poses,
                                               bool enabled)
{
  if (link2cow_.find(name) != link2cow_.end())
    removeCollisionObject(name);

  COW::Ptr new_cow = createFCLCollisionObject(name, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst& FCLDiscreteBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
FCLDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool FCLDiscreteBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool FCLDiscreteBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    fcl_co_count_ -= objects.size();

    std::vector<fcl::CollisionObject<double>*> static_objs;
    static_manager_->getObjects(static_objs);

    std::vector<fcl::CollisionObject<double>*> dynamic_objs;
    dynamic_manager_->getObjects(dynamic_objs);

    // Must check if object exists in the manager before calling unregister.
    // If it does not exist and unregister is called it is undefined behavior
    for (auto& co : objects)
    {
      auto static_it = std::find(static_objs.begin(), static_objs.end(), co.get());
      if (static_it != static_objs.end())
        static_manager_->unregisterObject(co.get());

      auto dynamic_it = std::find(dynamic_objs.begin(), dynamic_objs.end(), co.get());
      if (dynamic_it != dynamic_objs.end())
        dynamic_manager_->unregisterObject(co.get());
    }

    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), name));
    link2cow_.erase(name);
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::isCollisionObjectEnabled(const std::string& name) const
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
    if (!cur_tf.translation().isApprox(pose.translation(), 1e-8) || !cur_tf.rotation().isApprox(pose.rotation(), 1e-8))
    {
      it->second->setCollisionObjectsTransform(pose);
      if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        static_manager_->update(it->second->getCollisionObjectsRaw());
      }
      else
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        dynamic_manager_->update(it->second->getCollisionObjectsRaw());
      }
    }
  }
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                         const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  static_update_.clear();
  dynamic_update_.clear();
  for (auto i = 0U; i < names.size(); ++i)
  {
    auto it = link2cow_.find(names[i]);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
      if (!cur_tf.translation().isApprox(poses[i].translation(), 1e-8) ||
          !cur_tf.rotation().isApprox(poses[i].rotation(), 1e-8))
      {
        it->second->setCollisionObjectsTransform(poses[i]);
        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update_.insert(static_update_.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because FCL supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& transform : transforms)
  {
    auto it = link2cow_.find(transform.first);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not updated to prevent unnecessary re-balancing of the BVH tree
      if (!cur_tf.translation().isApprox(transform.second.translation(), 1e-8) ||
          !cur_tf.rotation().isApprox(transform.second.rotation(), 1e-8))
      {
        it->second->setCollisionObjectsTransform(transform.second);
        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update_.insert(static_update_.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because FCL supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

const std::vector<std::string>& FCLDiscreteBVHManager::getCollisionObjects() const { return collision_objects_; }

void FCLDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
    updateCollisionObjectFilters(active_, co.second, static_manager_, dynamic_manager_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

const std::vector<std::string>& FCLDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }

void FCLDiscreteBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  collision_margin_data_ = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& FCLDiscreteBVHManager::getCollisionMarginData() const { return collision_margin_data_; }

void FCLDiscreteBVHManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                       CollisionMarginPairOverrideType override_type)
{
  collision_margin_data_.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setDefaultCollisionMargin(double default_collision_margin)
{
  collision_margin_data_.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setCollisionMarginPair(const std::string& name1,
                                                   const std::string& name2,
                                                   double collision_margin)
{
  collision_margin_data_.setCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::incrementCollisionMargin(double increment)
{
  collision_margin_data_.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract_common::ContactAllowedValidator> validator)
{
  validator_ = std::move(validator);
}
std::shared_ptr<const tesseract_common::ContactAllowedValidator>
FCLDiscreteBVHManager::getContactAllowedValidator() const
{
  return validator_;
}

void FCLDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  ContactTestData cdata(active_, collision_margin_data_, validator_, request, collisions);
  if (collision_margin_data_.getMaxCollisionMargin() > 0)
  {
    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &cdata, &distanceCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&cdata, &distanceCallback);
  }
  else
  {
    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &cdata, &collisionCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&cdata, &collisionCallback);
  }
}

void FCLDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  std::size_t cnt = cow->getCollisionObjectsRaw().size();
  fcl_co_count_ += cnt;
  static_update_.reserve(fcl_co_count_);
  dynamic_update_.reserve(fcl_co_count_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    // If static add to static manager
    for (auto& co : objects)
      static_manager_->registerObject(co.get());
  }
  else
  {
    for (auto& co : objects)
      dynamic_manager_->registerObject(co.get());
  }

  // If active links is not empty update filters to replace the active links list
  if (!active_.empty())
    updateCollisionObjectFilters(active_, cow, static_manager_, dynamic_manager_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

void FCLDiscreteBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  for (auto& cow : link2cow_)
  {
    cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin() / 2.0);
    std::vector<CollisionObjectRawPtr>& co = cow.second->getCollisionObjectsRaw();
    if (cow.second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      static_update_.insert(static_update_.end(), co.begin(), co.end());
    }
    else
    {
      dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
    }
  }

  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}
}  // namespace tesseract_collision::tesseract_collision_fcl
