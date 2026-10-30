#ifndef EASYMESH_ASSOC_MANAGER_H
#define EASYMESH_ASSOC_MANAGER_H

#include "ns3/wifi-default-assoc-manager.h"
#include "ns3/mac48-address.h"

#include <set>

using namespace ns3;

// ────────────────────────────────────────────────────────────────
// EasyMeshAssocManager – association manager that can pin a STA to
// one BSS, modelling an 802.11v BSS Transition Management request
// (client steering) or an EasyMesh Backhaul Steering Request.
// The target is a set of BSSIDs: one for a single-link AP, every
// affiliated AP of the target AP MLD for a Wi-Fi 7 multi-link STA (the
// STA may hear the AP MLD on any of its links first; multi-link setup
// then adds the other links). Without a target it behaves like
// WifiDefaultAssocManager.
// ────────────────────────────────────────────────────────────────
class EasyMeshAssocManager : public WifiDefaultAssocManager
{
  public:
    static TypeId GetTypeId();

    void SetTargetBssid(Mac48Address bssid) { m_targets = {bssid}; }
    void SetTargetBssids(std::set<Mac48Address> bssids) { m_targets = std::move(bssids); }
    void ClearTargetBssid() { m_targets.clear(); }
    const std::set<Mac48Address>& GetTargetBssids() const { return m_targets; }

  protected:
    bool CanBeInserted(const StaWifiMac::ApInfo& apInfo) const override;
    bool CanBeReturned(const StaWifiMac::ApInfo& apInfo) const override;

  private:
    bool IsTarget(Mac48Address bssid) const;

    std::set<Mac48Address> m_targets;   // empty: no target
};

#endif // EASYMESH_ASSOC_MANAGER_H
