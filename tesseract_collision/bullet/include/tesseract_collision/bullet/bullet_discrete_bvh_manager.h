/**
 * @file bullet_discrete_bvh_manager.h
 * @brief Tesseract ROS Bullet discrete BVH collision manager.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
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
#ifndef TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H
#define TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H

#include <tesseract_collision/bullet/bullet_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/bullet/tesseract_collision_configuration.h>

namespace tesseract_collision::tesseract_collision_bullet
{
/** @brief A BVH implementation of a bullet manager */
class BulletDiscreteBVHManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<BulletDiscreteBVHManager>;
  using ConstPtr = std::shared_ptr<const BulletDiscreteBVHManager>;
  using UPtr = std::unique_ptr<BulletDiscreteBVHManager>;
  using ConstUPtr = std::unique_ptr<const BulletDiscreteBVHManager>;

  BulletDiscreteBVHManager(std::string name = "BulletDiscreteBVHManager",
                           TesseractCollisionConfigurationInfo config_info = TesseractCollisionConfigurationInfo());
  ~BulletDiscreteBVHManager() override;
  BulletDiscreteBVHManager(const BulletDiscreteBVHManager&) = delete;
  BulletDiscreteBVHManager& operator=(const BulletDiscreteBVHManager&) = delete;
  BulletDiscreteBVHManager(BulletDiscreteBVHManager&&) = delete;
  BulletDiscreteBVHManager& operator=(BulletDiscreteBVHManager&&) = delete;

  std::string getName() const override final;

  DiscreteContactManager::UPtr clone() const override final;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract_common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override final;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const override final;

  const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const override final;

  bool hasCollisionObject(const std::string& name) const override final;

  bool removeCollisionObject(const std::string& name) override final;

  bool enableCollisionObject(const std::string& name) override final;

  bool disableCollisionObject(const std::string& name) override final;

  bool isCollisionObjectEnabled(const std::string& name) const override final;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override final;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract_common::VectorIsometry3d& poses) override final;

  void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) override final;

  const std::vector<std::string>& getCollisionObjects() const override final;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override final;

  const std::vector<std::string>& getActiveCollisionObjects() const override final;

  void setCollisionMarginData(CollisionMarginData collision_margin_data) override final;

  const CollisionMarginData& getCollisionMarginData() const override final;

  void setCollisionMarginPairData(
      const CollisionMarginPairData& pair_margin_data,
      CollisionMarginPairOverrideType override_type = CollisionMarginPairOverrideType::REPLACE) override final;

  void setDefaultCollisionMargin(double default_collision_margin) override final;

  void setCollisionMarginPair(const std::string& name1,
                              const std::string& name2,
                              double collision_margin) override final;

  void incrementCollisionMargin(double increment) override final;

  void
  setContactAllowedValidator(std::shared_ptr<const tesseract_common::ContactAllowedValidator> validator) override final;

  std::shared_ptr<const tesseract_common::ContactAllowedValidator> getContactAllowedValidator() const override final;

  void contactTest(ContactResultMap& collisions, const ContactRequest& request) override final;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(const COW::Ptr& cow);

private:
  std::string name_;
  /** @brief A list of the active collision objects */
  std::vector<std::string> active_;
  /** @brief A list of the collision objects */
  std::vector<std::string> collision_objects_;
  /** @brief The bullet collision dispatcher used for getting object to object collison algorithm */
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
  /** @brief The bullet collision dispatcher configuration information */
  btDispatcherInfo dispatch_info_;
  /** @brief The bullet collision configuration information */
  TesseractCollisionConfigurationInfo config_info_;
  /** @brief The bullet collision configuration */
  TesseractCollisionConfiguration coll_config_;
  /** @brief The bullet broadphase interface */
  std::unique_ptr<btBroadphaseInterface> broadphase_;
  /** @brief A map of all (static and active) collision objects being managed */
  Link2Cow link2cow_;

  /**
   * @brief This is used when contactTest is called. It is also added as a user point to the collsion objects
   * so it can be used to exit collision checking for compound shapes.
   */
  ContactTestData contact_test_data_;

  /** @brief Filter collision objects before broadphase check */
  TesseractOverlapFilterCallback broadphase_overlap_cb_;

  /** @brief This function will update internal data when margin data has changed */
  void onCollisionMarginDataChanged();
};

}  // namespace tesseract_collision::tesseract_collision_bullet
#endif  // TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H
