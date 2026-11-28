#pragma once
#include "recovery_adapters.hpp"

namespace harness::observation {
struct RecoveryChannelMessage {
  Json payload;
  Ns received_monotonic_ns = -1;
};
// Receipt time and controller ownership are measured locally by the transport,
// never accepted as assertions in a remote JSON message.
struct RecoveryWorkcellChannel {
  virtual ~RecoveryWorkcellChannel() = default;
  virtual bool configure(const Json &, const Json &) = 0;
  virtual RecoveryChannelMessage state() = 0;
  virtual RecoveryChannelMessage acknowledgement() = 0;
  virtual bool owns(const std::string &, const Json &) = 0;
  virtual bool send(const Json &) = 0;
};

class ChannelRecoveryEnvironment final : public RecoveryEnvironment {
  RecoveryWorkcellChannel &channel_;
  Json config_, binding_, transaction_, staged_, committed_, contact_;
  Observations observations_;
  RecoveryChannelMessage measured_, acknowledgement_;
  double max_age_s_ = 0;
  bool configured_ = false, observed_ = false;
  Ns context_sent_monotonic_ns_ = -1;
  Json fresh(const RecoveryChannelMessage &) const;
  bool resources_owned();
public:
  explicit ChannelRecoveryEnvironment(RecoveryWorkcellChannel &channel) : channel_(channel) {}
  bool configure(const RecoveryRecipe &, const Json &, const Json &) override;
  void observe(const Observations &) override;
  RecoveryEvidence test(const std::string &) override;
  Json measured_state() const override;
  bool owns(const std::string &) const override;
  bool quiescent() const override;
  bool prepare_client(const Json &, const Json &) override;
  Json client_acknowledgement() override;
  bool validate_candidate(const Json &, const Json &) override;
  bool commit_candidate(const Json &) override;
  void handback_confirmed(const Json &) override;
  bool cancel(const std::string &) override;
  bool start_contact(const RecoveryStage &) override;
  std::string poll_contact() override;
  bool contact_quiescent() override;
};
} // namespace harness::observation
