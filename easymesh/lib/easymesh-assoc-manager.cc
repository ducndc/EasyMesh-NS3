#include "easymesh-assoc-manager.h"

NS_OBJECT_ENSURE_REGISTERED(EasyMeshAssocManager);

TypeId
EasyMeshAssocManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::EasyMeshAssocManager")
                            .SetParent<WifiDefaultAssocManager>()
                            .AddConstructor<EasyMeshAssocManager>();
    return tid;
}

bool
EasyMeshAssocManager::IsTarget(Mac48Address bssid) const
{
    return m_targets.empty() || m_targets.count(bssid) > 0;
}

bool
EasyMeshAssocManager::CanBeInserted(const StaWifiMac::ApInfo& apInfo) const
{
    return WifiDefaultAssocManager::CanBeInserted(apInfo) && IsTarget(apInfo.m_bssid);
}

bool
EasyMeshAssocManager::CanBeReturned(const StaWifiMac::ApInfo& apInfo) const
{
    return IsTarget(apInfo.m_bssid);
}
