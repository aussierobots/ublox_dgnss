#ifndef UBLOX_DGNSS_NODE__UBX__UBX_MON_COMMS_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_MON_COMMS_HPP_

#include <cstring>
#include <vector>
#include <tuple>
#include <string>
#include <sstream>
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx::mon::comms {

struct PortInfo {
    u2_t portId;
    u2_t txPending;
    u4_t txBytes;
    u1_t txUsage;
    u1_t txPeakUsage;
    u2_t rxPending;
    u4_t rxBytes;
    u1_t rxUsage;
    u1_t rxPeakUsage;
    u2_t overrunErrs;
    u2_t msgs[4];
    u4_t skipped;
};

class MonCommsPayload : UBXPayload {
public:
    static const msg_class_t MSG_CLASS = UBX_MON;
    static const msg_id_t MSG_ID = UBX_MON_COMMS;

    u1_t version;
    u1_t nPorts;
    x1_t txErrors;
    u1_t reserved0;
    u1_t protIds[4];
    std::vector<PortInfo> ports;

public:
    MonCommsPayload() : UBXPayload(MSG_CLASS, MSG_ID) {}

    MonCommsPayload(u1_t * payload_polled, u2_t size) : UBXPayload(MSG_CLASS, MSG_ID) {
        payload_.clear();
        payload_.reserve(size);
        payload_.resize(size);
        memcpy(payload_.data(), payload_polled, size);

        auto ptr = payload_.data();
        memcpy(&version, ptr, sizeof(version));
        ptr += sizeof(version);
        memcpy(&nPorts, ptr, sizeof(nPorts));
        ptr += sizeof(nPorts);
        memcpy(&txErrors, ptr, sizeof(txErrors));
        ptr += sizeof(txErrors);
        ptr += sizeof(reserved0);  // skip reserved0
        memcpy(protIds, ptr, sizeof(protIds));
        ptr += sizeof(protIds);

        for (int i = 0; i < nPorts; ++i) {
            PortInfo info;
            memcpy(&info, ptr, sizeof(PortInfo));
            ports.push_back(info);
            ptr += sizeof(PortInfo);
        }
    }

    std::tuple<u1_t *, size_t> make_poll_payload() override {
        payload_.clear();
        return std::make_tuple(payload_.data(), payload_.size());
    }

    std::string to_string() {
        std::ostringstream oss;
        oss << "version: " << static_cast<int>(version);
        oss << " nPorts: " << static_cast<int>(nPorts);
        oss << " txErrors: " << static_cast<int>(txErrors);
        for (auto &port : ports) {
            oss << " PortID: " << port.portId << " TxPending: " << port.txPending << " TxBytes: " << port.txBytes;
        }
        return oss.str();
    }
};

}  // namespace ubx::mon::comms

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_MON_COMMS_HPP_
