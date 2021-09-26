#pragma once

#include <modm/board.hpp>

#include <etl/array.h>
#include <etl/basic_format_spec.h>
#include <etl/delegate.h>
#include <etl/hash.h>
#include <etl/unordered_map.h>
#include <etl/utility.h>

using namespace Board;

namespace ublox {

template <typename E> constexpr auto to_underlying(E e) noexcept { return static_cast<std::underlying_type_t<E>>(e); }
struct PairHash {
    template <class T1, class T2> std::size_t operator()(const etl::pair<T1, T2>& pair) const {
        return etl::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

__attribute__((__packed__)) typedef struct ubx_frame_sync {
    uint8_t syncA;
    uint8_t syncB;
} ubx_frame_sync_t;
__attribute__((__packed__)) typedef struct ubx_frame_header {
    ubx_frame_sync_t sync;
    uint8_t class_;
    uint8_t id;
    uint16_t length;
    // payload
    // uint8_t crcA_;
    // uint8_t crcB;
} ubx_frame_header_t;
__attribute__((__packed__)) typedef struct ubx_frame_crc {
    uint8_t a;
    uint8_t b;
} ubx_frame_crc_t;

// __attribute__((__packed__)) typedef struct ubx_frame {
//     ubx_frame_header_t header_;
//     uint8_t payload_[256];

// } ubx_frame_t;

/// get some structs from here:
/// http://apm-docs.info/libraries/AP__GPS__UBLOX_8h_source.html#l00242
/// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

__attribute__((__packed__)) typedef struct ubx_nav_pvt {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t t_acc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t flags2;
    uint8_t num_sv;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t h_msl;
    uint32_t h_acc;
    uint32_t v_acc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gspeed;
    int32_t head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t p_dop;
    uint8_t reserved1[6];
    uint32_t headVeh;
    uint8_t reserved2[4];
} ubx_nav_pvt_t;

__attribute__((__packed__)) typedef struct ubx_nav_status {
    // [ms] GPS time of week of the navigation epoch
    uint32_t iTOW;
    // GPSfix Type, this value does not qualify a fix as valid and within the limits. See note on flag
    // gpsFixOk below. 0x00 = no fix, 0x01 = dead reckoning only, 0x02 = 2D-fix, 0x03 = 3D-fix, 0x04 =
    // GPS + dead reckoning combined, 0x05 = Time only fix, 0x06..0xff = reserved
    uint8_t gpsFix;
    // [0] gpsFixOk 1 = position and velocity valid and within DOP and ACC Masks.
    // [1] diffSoln 1 = differential corrections were applied
    // [2] wknSet 1 = Week Number valid (see Time Validity section for details)
    // [3] towSet 1 = Time of Week valid (see Time Validity section for details)
    uint8_t flags;
    // [0] diffCorr 1 = differential corrections available
    // [1] carrSolnValid 1 = valid carrSoln
    // [6,7] mapMatching map matching status:
    //	00: none
    // 	01: valid but not used, i.e. map matching data was received, but was too old
    // 	10: valid and used, map matching data was applied
    // 	11: valid and used, map matching data was applied. In case of sensor unavailability map matching
    // data enables dead reckoning. This requires map matched latitude/longitude or heading data
    uint8_t fixStat;
    // [0,1] psmState power save mode state
    // 0: ACQUISITION [or when psm disabled]
    // 1: TRACKING
    // 2: POWER OPTIMIZED TRACKING
    // 3: INACTIVE
    // [3,4] spoofDetState Spoofing detection state (not supported in protocol versions less than 18)
    // 0: Unknown or deactivated
    // 1: No spoofing indicated
    // 2: Spoofing indicated
    // 3: Multiple spoofing indications
    // Note that the spoofing state value only reflects the detector state for the current navigation epoch.
    // As spoofing can be detected most easily at the transition from real signal to spoofing signal, this is
    // also where the detector is triggered the most. I.e. a value of 1 - No spoofing indicated does not mean
    // that the receiver is not spoofed, it simply states that the detector was not triggered in this epoch.
    // [6,7] carrSoln Carrier phase range solution status:
    // 0: no carrier phase range solution
    // 1: carrier phase range solution with floating ambiguities
    // 2: carrier phase range solution with fixed ambiguities
    uint8_t flags2;
    // [ms] Time to first fix (millisecond time tag)
    uint32_t ttff;
    // [ms] Milliseconds since Startup / Reset
    uint32_t msss;
} ubx_nav_status_t;

__attribute__((__packed__)) typedef struct ubx_nav_clock {
    // [ms] GPS  time  of  week  of  the  navigation  epoch.
    uint32_t iTOW;
    // [ns] clock bias
    int32_t clkB;
    // [ns/s] clock drift
    int32_t clkD;
    // [ns] time accuracy estimate
    uint32_t tAcc;
    // [ps/s] frequency accuracy estimate
    uint32_t fAcc;
} ubx_nav_clock_t;

__attribute__((__packed__)) typedef struct ubx_nav_cov {
    // [ms] GPS  time  of  week  of  the  navigation  epoch.
    uint32_t iTOW;
    // message version (0x00 for version 8)
    uint8_t version;
    // position covariance matrix validity flag
    uint8_t posCovValid;
    // velocity covariance matrix validity flag
    uint8_t velCovValid;
    // reserved
    uint8_t reserved[9];
    // [m²] position covariance matrix for north, east, down
    uint32_t posCovNN;
    uint32_t posCovNE;
    uint32_t posCovND;
    uint32_t posCovEE;
    uint32_t posCovED;
    uint32_t posCovDD;
    // [m²/s²] speed covariance matrix for north, east, down
    uint32_t velCovNN;
    uint32_t velCovNE;
    uint32_t velCovND;
    uint32_t velCovEE;
    uint32_t velCovED;
    uint32_t velCovDD;
} ubx_nav_cov_t;

enum class UbxMessageClass : uint8_t { NONE = 0x00, NAV = 0x01 };

enum class UbxNavMessageId : uint8_t {
    NONE = 0x00,
    UBX_NAV_PVT = 0x07,
    UBX_NAV_STATUS = 0x03,
    UBX_NAV_CLOCK = 0x22,
    UBX_NAV_COV = 0x36
};

class UbloxDriver {
    static constexpr size_t SIZE = 1024;

    template <typename T> using UserDataReceiveCallback = etl::delegate<void(T)>;

    using NavStatusCallback = UserDataReceiveCallback<ubx_nav_status_t>;
    using NavPvtCallback = UserDataReceiveCallback<ubx_nav_pvt_t>;
    using NavClockCallback = UserDataReceiveCallback<ubx_nav_clock_t>;
    using NavCovCallback = UserDataReceiveCallback<ubx_nav_cov_t>;

    using Buffer = etl::array<uint8_t, SIZE>;

    using P = const UbloxDriver&;
    using DecodeFunction = etl::delegate<void(P, const uint8_t*, const uint16_t)>;
    typedef struct DecodeLookupData {
        DecodeFunction decodeFunction;
    } DecodeLookupData_t;
    using UbxDecodeLookupKey_t = etl::pair<uint8_t, uint8_t>;
    using UbxDecodeLookupDict_t = etl::unordered_map<UbxDecodeLookupKey_t, DecodeLookupData, 32, 32, PairHash>;

public:
    UbloxDriver()
        : state_(STATE::WAITING_FOR_SYNC_A)
        , byteCount_(0)
        , readEnable_(false)
        , buf_ { 0 } { }

    void next(uint8_t byte) {
        // MODM_LOG_INFO << "B: 0x" << modm::hex << byte << modm::endl;
        if (state_ == STATE::WAITING_FOR_SYNC_A && byte == to_underlying(SPECIAL::SYNC_A)) {
            MODM_LOG_INFO << "========================================" << modm::endl;
            /// check if we have a full frame together from last sync
            /// header + crc = 8 bytes, so if our supposed frame is less than that, it is definiteley corrupted
            if (byteCount_ > sizeof(ubx_frame_header_t) + sizeof(ubx_frame_crc_t)) {
                processFrame(buf_);
                byteCount_ = 0;
            }
            // MODM_LOG_INFO << "SYNC_A" << modm::endl;
            readEnable_ = true;
            state_ = STATE::WAITING_FOR_SYNC_B;
        } else if (state_ == STATE::WAITING_FOR_SYNC_B && byte == to_underlying(SPECIAL::SYNC_B)) {
            state_ = STATE::WAITING_FOR_SYNC_A;
            // MODM_LOG_INFO << "SYNC_B" << modm::endl;
        }

        if (readEnable_) {
            if (byteCount_ < buf_.size()) {
                buf_.at(byteCount_) = byte;
                byteCount_++;
            } else {
                //   buffer overflow because syncs were not received
                // reset
                MODM_LOG_INFO << "CRITICAL PROBLEM WHILE BUFFERING ... [byteCount: " << byteCount_ << "]" << modm::endl;
                byteCount_ = 0;
                readEnable_ = false;
                state_ = STATE::WAITING_FOR_SYNC_A;
            }
        }
    }

    void processFrame(const Buffer& buf) {
        /// grab header
        const ubx_frame_header_t& frameHeader = *reinterpret_cast<const ubx_frame_header_t*>(buf.data());
        // length field of the header seems incosistent with the message definition ..
        // it should be 16 for UBX_NAVSTATUS, but the ublox sends 20 ...
        // they probably cound the class + id + crcA + crcB as well, which means the actual size is whatever ublox sends
        // us -4
        /// frameHeader.length contains the sent payload length
        /// byteCount contains the actual received byte length, we should trust this more
        const uint16_t actualPayloadLength = byteCount_ - sizeof(ubx_frame_header_t) - sizeof(ubx_frame_crc_t);

        // check crc
        /// grab whole frame, but leave the syncs and the crc out
        const uint16_t crcFrameStartIndex = sizeof(ubx_frame_sync_t);
        const uint16_t crcFrameEndIndex = byteCount_ - sizeof(ubx_frame_sync_t);
        const ubx_frame_crc_t crc = *reinterpret_cast<const ubx_frame_crc_t*>(&buf.at(byteCount_ - sizeof(uint16_t)));
        ubx_frame_crc_t calculatedCrc = calculateCrc(buf, crcFrameStartIndex, crcFrameEndIndex);
        if (crc.a == calculatedCrc.a && crc.b == calculatedCrc.b) {
            /// crc ok
            /// grab payload
            const uint8_t* payload = &buf.at(sizeof(ubx_frame_header_t));
            /// do decoding
            auto it = decodeLookupDict.find(UbxDecodeLookupKey_t { frameHeader.class_, frameHeader.id });
            if (it != decodeLookupDict.end()) {
                if (it->second.decodeFunction.is_valid()) {
                    it->second.decodeFunction(*this, payload, actualPayloadLength);
                }
            } else {
                printFrameHeader(frameHeader);
            }

        } else {
            /// drop boken packages
        }
    }

    ubx_frame_crc_t calculateCrc(const Buffer& buf, uint16_t start, uint16_t end) {
        uint8_t a = 0;
        uint8_t b = 0;
        for (uint16_t i = start; i < end; i++) {
            a = a + buf.at(i);
            b = b + a;
        }
        return ubx_frame_crc_t { a, b };
    }

    template <typename T, typename CB> inline void userCall(const T* ptr, const uint16_t length, const CB cb) const {
        /// does this frame and the userCallback seem ok?
        if (length == sizeof(T) && cb.is_valid()) {
            /// create copy of the payload data before calling user callbacks
            T copy;
            std::memcpy(&copy, ptr, length);
            /// call user callback
            cb(copy);
        }
    }
    template <typename T> inline const T* payloadCast(const uint8_t* payload) const {
        const T* ptr = reinterpret_cast<const T*>(payload);
        return ptr;
    }

    void printFrameHeader(const ubx_frame_header_t& header) {
        MODM_LOG_INFO << "FrameHeader:" << modm::endl
                      << modm::hex << "  - class: 0x" << modm::hex << header.class_ << modm::endl
                      << "  - id: 0x" << modm::hex << header.id << modm::endl
                      << "  - length: " << header.length << modm::endl;
    }

    // void setNavStatus(const ubx_nav_status_t& x) { navStatus_ = x; }
    // const ubx_nav_status_t& getNavStatus() const { return navStatus_; }
    // void setNavPvt(ubx_nav_pvt_t x) { navPvt_ = x; }
    // const ubx_nav_pvt_t& getNavPvt() const { return navPvt_; }
    // void setNavClock(ubx_nav_clock_t x) { navClock_ = x; }
    // const ubx_nav_clock_t& getNavClock() const { return navClock_; }

    void registerNavStatusCallback(NavStatusCallback cb) { navStatusCb_ = cb; }
    void registerNavPvtCallback(NavPvtCallback cb) { navPvtCb_ = cb; }
    void registerNavClockCallback(NavClockCallback cb) { navClockCb_ = cb; }
    void registerNavCovCallback(NavCovCallback cb) { navCovCb_ = cb; }

private:
    enum class STATE { WAITING_FOR_SYNC_A, WAITING_FOR_SYNC_B };
    enum class SPECIAL : uint8_t { SYNC_A = 0xb5, SYNC_B = 0x62 };

public:
    // callbacks for ubx navigation receive events
    NavStatusCallback navStatusCb_;
    NavPvtCallback navPvtCb_;
    NavClockCallback navClockCb_;
    NavCovCallback navCovCb_;

private:
    STATE state_;
    size_t byteCount_;
    bool readEnable_;
    // uint8_t buf_[SIZE];
    Buffer buf_;

    /// internal buffers for ubx navigation information
    // ubx_nav_status_t navStatus_;
    // ubx_nav_pvt_t navPvt_;
    // ubx_nav_clock_t navClock_;
    // ubx_nav_clock_t navCov_;

    /// dictionary containing class & id to decode function specifications
    static inline UbxDecodeLookupDict_t decodeLookupDict {
        { etl::make_pair(to_underlying(UbxMessageClass::NAV), to_underlying(UbxNavMessageId::UBX_NAV_PVT)),
          DecodeLookupData_t { [](P parent, const uint8_t* payload, const uint16_t length) {
              parent.userCall(parent.payloadCast<ubx_nav_pvt_t>(payload), length, parent.navPvtCb_);
          } } },
        { etl::make_pair(to_underlying(UbxMessageClass::NAV), to_underlying(UbxNavMessageId::UBX_NAV_STATUS)),
          DecodeLookupData_t { [](P parent, const uint8_t* payload, const uint16_t length) {
              parent.userCall(parent.payloadCast<ubx_nav_status_t>(payload), length, parent.navStatusCb_);
          } } },
        { etl::make_pair(to_underlying(UbxMessageClass::NAV), to_underlying(UbxNavMessageId::UBX_NAV_CLOCK)),
          DecodeLookupData_t { [](P parent, const uint8_t* payload, const uint16_t length) {
              parent.userCall(parent.payloadCast<ubx_nav_clock_t>(payload), length, parent.navClockCb_);
          } } },
        { etl::make_pair(to_underlying(UbxMessageClass::NAV), to_underlying(UbxNavMessageId::UBX_NAV_COV)),
          DecodeLookupData_t { [](P parent, const uint8_t* payload, const uint16_t length) {
              parent.userCall(parent.payloadCast<ubx_nav_cov_t>(payload), length, parent.navCovCb_);
          } } }
    };
};

} // namespace ublox