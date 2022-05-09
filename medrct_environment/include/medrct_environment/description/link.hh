#pragma once

#include <memory>
#include <vector>

namespace medrct
{
namespace env
{
struct Visual
{
};

struct Collision
{
};

class Link
{
public:
  using Ptr = std::shared_ptr<Link>;
  using ConstPtr = std::shared_ptr<const Link>;

  Link(const std::string& name) : name(name) {}
  Link() = default;
  ~Link() = default;

  Link(const Link& other) = default;
  Link& operator=(const Link& other) = default;

  Link(Link&& other) = default;
  Link& operator=(Link&& other) = default;

  const std::string& getName() const { return name; }

  /// Visual Elements
  std::vector<Visual> visual;

  /// Collision Elements
  std::vector<Collision> collision;

private:
  std::string name;
};
} // namespace env
} // namespace medrct
