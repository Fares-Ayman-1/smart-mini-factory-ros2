// GraspAttach: a simulation-only grasp assist for the factory arm.
//
// Friction-only grasping of small parts in Gazebo is unreliable (products slip
// or get squeezed out), so when the arm has closed its fingers it asks this
// system to lock the product to the gripper with a fixed joint, and to release
// it again when the fingers open.
//
// The lock is only made if a product really sits between the finger pads:
// its centre must be within `max_offset` of the TCP across and along the pads
// and within `max_dz` along the gripper axis. A badly placed grasp therefore
// still fails, so vision and IK accuracy still matter.
//
// Topics (gz transport):
//   <attach_topic>  gz.msgs.Empty      lock the product between the fingers
//   <detach_topic>  gz.msgs.Empty      release it
//   <state_topic>   gz.msgs.StringMsg  name of the held product, or "none"

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace factory
{
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;

class GraspAttach : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    linkName_ = sdf->Get<std::string>("gripper_link", "gripper").first;
    tcp_ = sdf->Get<gz::math::Vector3d>("tcp_offset", gz::math::Vector3d(0.065, 0, 0)).first;
    maxOffset_ = sdf->Get<double>("max_offset", 0.012).first;
    maxDz_ = sdf->Get<double>("max_dz", 0.020).first;
    std::istringstream prefixes(
      sdf->Get<std::string>("prefixes", "product_ trial_").first);
    for (std::string p; prefixes >> p;) prefixes_.push_back(p);

    const auto attach = sdf->Get<std::string>("attach_topic", "/factory/grasp/attach").first;
    const auto detach = sdf->Get<std::string>("detach_topic", "/factory/grasp/detach").first;
    const auto state = sdf->Get<std::string>("state_topic", "/factory/grasp/state").first;
    node_.Subscribe(attach, &GraspAttach::OnAttach, this);
    node_.Subscribe(detach, &GraspAttach::OnDetach, this);
    statePub_ = node_.Advertise<gz::msgs::StringMsg>(state);
    gzmsg << "[GraspAttach] gripper link [" << linkName_ << "], attach on [" << attach
          << "], detach on [" << detach << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    if (gripper_ == kNullEntity) {
      gripper_ = model_.LinkByName(ecm, linkName_);
      if (gripper_ == kNullEntity) return;
    }
    bool attach = false, detach = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::swap(attach, attachReq_);
      std::swap(detach, detachReq_);
    }
    if (detach && joint_ != kNullEntity) {
      ecm.RequestRemoveEntity(joint_);
      joint_ = kNullEntity;
      held_ = "none";
      Publish();
    }
    if (attach) {
      if (joint_ == kNullEntity) TryAttach(ecm);
      Publish();
    }
  }

private:
  void OnAttach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    attachReq_ = true;
  }

  void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    detachReq_ = true;
  }

  bool Matches(const std::string &name) const
  {
    for (const auto &p : prefixes_)
      if (name.rfind(p, 0) == 0) return true;
    return false;
  }

  void TryAttach(EntityComponentManager &ecm)
  {
    const auto tcp = gz::sim::worldPose(gripper_, ecm) *
                     gz::math::Pose3d(tcp_, gz::math::Quaterniond::Identity);
    Entity bestLink = kNullEntity;
    std::string bestName;
    double bestDist = 1e9;
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const Entity &e, const gz::sim::components::Model *,
          const gz::sim::components::Name *name) -> bool {
        if (!Matches(name->Data())) return true;
        const auto links = gz::sim::Model(e).Links(ecm);
        if (links.empty()) return true;
        // product centre expressed in the TCP frame:
        // x = along the gripper axis, y = across the pads (closing), z = along the pads
        const auto rel = tcp.Inverse() * gz::sim::worldPose(links.front(), ecm);
        const auto &p = rel.Pos();
        if (std::abs(p.X()) > maxDz_ || std::abs(p.Y()) > maxOffset_ ||
            std::abs(p.Z()) > maxOffset_) {
          return true;
        }
        const double d = p.Length();
        if (d < bestDist) {
          bestDist = d;
          bestLink = links.front();
          bestName = name->Data();
        }
        return true;
      });
    if (bestLink == kNullEntity) {
      held_ = "none";
      return;
    }
    joint_ = ecm.CreateEntity();
    ecm.CreateComponent(joint_, gz::sim::components::DetachableJoint(
                                  {gripper_, bestLink, "fixed"}));
    held_ = bestName;
    gzmsg << "[GraspAttach] holding [" << bestName << "], "
          << bestDist * 1000 << " mm from the TCP\n";
  }

  void Publish()
  {
    gz::msgs::StringMsg msg;
    msg.set_data(held_);
    statePub_.Publish(msg);
  }

  gz::sim::Model model_{kNullEntity};
  std::string linkName_;
  gz::math::Vector3d tcp_;
  double maxOffset_{0.012};
  double maxDz_{0.020};
  std::vector<std::string> prefixes_;
  Entity gripper_{kNullEntity};
  Entity joint_{kNullEntity};
  std::string held_{"none"};
  gz::transport::Node node_;
  gz::transport::Node::Publisher statePub_;
  std::mutex mutex_;
  bool attachReq_{false};
  bool detachReq_{false};
};
}  // namespace factory

GZ_ADD_PLUGIN(factory::GraspAttach, gz::sim::System,
              factory::GraspAttach::ISystemConfigure,
              factory::GraspAttach::ISystemPreUpdate)
