#ifndef EASYMESH_ASSOC_MANAGER_H
#define EASYMESH_ASSOC_MANAGER_H

#include "ns3/wifi-default-assoc-manager.h"
#include "ns3/mac48-address.h"

#include <optional>

using namespace ns3;

// ────────────────────────────────────────────────────────────────
// EasyMeshAssocManager – association manager that can pin a STA to
// one BSSID, modelling an 802.11v BSS Transition Management request
// (client steering) or an EasyMesh Backhaul Steering Request.
// Without a target it behaves like WifiDefaultAssocManager.
// ────────────────────────────────────────────────────────────────
class EasyMeshAssocManager : public WifiDefaultAssocManager
{
  public:
    static TypeId GetTypeId();

    void SetTargetBssid(Mac48Address bssid) { m_target = bssid; }
    void ClearTargetBssid() { m_target.reset(); }
    std::optional<Mac48Address> GetTargetBssid() const { return m_target; }

  protected:
    bool CanBeInserted(const StaWifiMac::ApInfo& apInfo) const override;
    bool CanBeReturned(const StaWifiMac::ApInfo& apInfo) const override;

  private:
    std::optional<Mac48Address> m_target;
};

#endif // EASYMESH_ASSOC_MANAGER_H
