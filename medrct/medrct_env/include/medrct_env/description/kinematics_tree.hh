#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "link.hh"
#include "joint.hh"

namespace medrct
{
namespace env
{
// This class came from tesseract/tesseract_scene_graph SceneGraph
// and removed unneeded methods.
// https://github.com/tesseract-robotics/tesseract
class KinematicsTree
{
public:
  using Ptr = std::shared_ptr<KinematicsTree>;
  using ConstPtr = std::shared_ptr<const KinematicsTree>;

  KinematicsTree(const std::string& name = "");
  ~KinematicsTree() = default;

  KinematicsTree(const KinematicsTree& other) = delete;
  KinematicsTree& operator=(const KinematicsTree& other) = delete;

  KinematicsTree(KinematicsTree&& other) noexcept;
  KinematicsTree& operator=(KinematicsTree&& other) noexcept;

  /**
   * @brief Sets the kinematics tree name
   * @param name The name of the kinematics tree
   */
  void setName(const std::string& name);

  /**
   * @brief Sets the kinematics tree name
   * @param name The name of the kinematics tree
   */
  const std::string& getName() const;

  /**
   * @brief Gets the root link name (aka. World Coordinate Frame)
   * @return The root link name
   */
  const std::string& getRoot() const;

  /**
   * @brief Adds a link to the kinematics tree
   *
   * The first link added to the kinematics tree is set as the root by
   * default.
   *
   * @param link The link to be added to the kinematics tree
   * @return Return False if a link with the same name already exists
   */
  bool addLink(const Link& link);

  /**
   * @brief Get a link in the kinematics tree
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer
   * to the link
   */
  Link::ConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get a vector links in the kinematics tree
   * @return A vector of links
   */
  std::vector<Link::ConstPtr> getLinks() const;

  /**
   * @brief Adds joint to the kinematics tree
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint
   * name already exists in the kinematics tree, otherwise true. TODO more
   * errors
   */
  bool addJoint(const Joint& joint);

  /**
   * @brief Get a joint in the kinematics tree
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer
   * to the joint
   */
  Joint::ConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Get a vector of joints in the kinematics tree
   * @return A vector of joints
   */
  std::vector<Joint::ConstPtr> getJoints() const;

  /**
   * @brief Get a vector of active joints in the kinematics tree
   * @return A vector of active joints
   */
  std::vector<Joint::ConstPtr> getActiveJoints() const;

  bool setRoot(const std::string& link_name);

  /**
   * @brief Get a vector of joint names in order of the chain
   * @param chain_joint_names Vector of joint names of the chain from root to
   * tip
   * @return true if chain is found, false otherwise
   */
  bool getChainDescription(
      std::vector<std::string>& chain_joint_names,
      const std::string& root_link_name,
      const std::string& tip_link_name) const;

private:
  std::string name = "N/A";
  std::string tree_root_link_name = "N/A";
  std::unordered_map<std::string, Link::ConstPtr> link_map;
  std::unordered_map<std::string, Joint::ConstPtr> joint_map;

  std::unordered_map<std::string, std::string> link_to_parent_link;
  std::unordered_map<std::string, std::string> link_to_parent_joint;
};
} // namespace env
} // namespace medrct
