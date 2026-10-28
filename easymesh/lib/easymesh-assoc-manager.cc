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
EasyMeshAssocManager::CanBeInserted(const StaWifiMac::ApInfo& apInfo) const
{
    return WifiDefaultAssocManager::CanBeInserted(apInfo) &&
           (!m_target || apInfo.m_bssid == *m_target);
}

bool
EasyMeshAssocManager::CanBeReturned(const StaWifiMac::ApInfo& apInfo) const
{
    return !m_target || apInfo.m_bssid == *m_target;
}
