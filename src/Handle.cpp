#include "Handle.h"

#include "USBCAN_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

#include <cstring>
#include <iostream>
#include <thread>

namespace
{

// All protocol reads below intentionally use memcpy-based little-endian helpers
// instead of pointer reinterpret casts. This avoids the old strict-aliasing and
// unaligned-access UB that previously existed in packet parsing.
uint16_t ReadLeU16(const std::vector<uint8_t>& data, std::size_t offset)
{
    if (offset + sizeof(uint16_t) > data.size())
    {
        return 0;
    }

    uint16_t value = 0;
    std::memcpy(&value, data.data() + offset, sizeof(value));
    return value;
}

uint32_t ReadLeU32(const std::vector<uint8_t>& data, std::size_t offset)
{
    if (offset + sizeof(uint32_t) > data.size())
    {
        return 0;
    }

    uint32_t value = 0;
    std::memcpy(&value, data.data() + offset, sizeof(value));
    return value;
}

int16_t ReadLeI16(const std::vector<uint8_t>& data, std::size_t offset)
{
    if (offset + sizeof(int16_t) > data.size())
    {
        return 0;
    }

    int16_t value = 0;
    std::memcpy(&value, data.data() + offset, sizeof(value));
    return value;
}

void LogDataHandleLifecycle(const char* pszStage, const DJIR_SDK::DataHandle* pSelf)
{
#if defined(_DEBUG) && defined(_DEBUG_LIFECYCLE)
    std::cerr << "[Lifecycle][DJIR_SDK::DataHandle] " << pszStage
              << " this=" << pSelf
              << " thread=" << std::this_thread::get_id()
              << std::endl;
#else
    (void)pszStage;
    (void)pSelf;
#endif
}

} // namespace

DJIR_SDK::DataHandle::DataHandle(void *pCanConnection)
    : _dev(pCanConnection)
{
    LogDataHandleLifecycle("ctor begin", this);
    LogDataHandleLifecycle("ctor end", this);
}

DJIR_SDK::DataHandle::~DataHandle()
{
    LogDataHandleLifecycle("dtor begin", this);
    // Destruction always funnels through stop() so the worker lifecycle stays
    // identical whether shutdown is explicit or implicit.
    stop();
    LogDataHandleLifecycle("dtor end", this);
}

void DJIR_SDK::DataHandle::start()
{
    LogDataHandleLifecycle("start begin", this);
    if (_thread.joinable())
    {
        // The SDK currently assumes a single worker per DataHandle instance.
        // Re-start while running is treated as a no-op instead of spawning a
        // second parser thread over the same CAN stream.
        LogDataHandleLifecycle("start end", this);
        return;
    }

    _stopped.store(false);
    _thread = std::thread(&DataHandle::run, this);
    LogDataHandleLifecycle("start end", this);
}

void DJIR_SDK::DataHandle::stop()
{
    LogDataHandleLifecycle("stop begin", this);
    _stopped.store(true);
    if (_thread.joinable())
    {
        _thread.join();
    }
    LogDataHandleLifecycle("stop end", this);
}

void DJIR_SDK::DataHandle::add_cmd(std::vector<uint8_t> cmd)
{
    if (cmd.size() < 10)
    {
        // Too short to contain the protocol header CRC used for response matching.
        return;
    }

    TPendingCmd stPending;
    stPending.vCmd = std::move(cmd);
    stPending.uiHeaderCrc = ReadLeU16(stPending.vCmd, 8);

    std::lock_guard<std::mutex> lock(_rdcontent_lock);
    _vecPendingCmds.push_back(std::move(stPending));
    constexpr std::size_t kMaxPendingCmds = 32;
    while (_vecPendingCmds.size() > kMaxPendingCmds)
    {
        // Keep pending state bounded. If responses are lost or mismatched we
        // prefer dropping the oldest stale expectation over unbounded growth.
        _vecPendingCmds.pop_front();
    }
}

bool DJIR_SDK::DataHandle::get_position(int16_t &yaw, int16_t &roll, int16_t &pitch, uint16_t timeout_ms)
{
    std::unique_lock<std::mutex> lk(_input_position_mutex);
    const bool bReady = _input_position_cond_var.wait_for(
        lk,
        std::chrono::milliseconds(timeout_ms),
        [this]()
        {
            return _stPositionSample.bReady;
        });

    if (!bReady)
    {
        return false;
    }

    yaw = _stPositionSample.nYaw;
    roll = _stPositionSample.nRoll;
    pitch = _stPositionSample.nPitch;
    _stPositionSample.bReady = false;
    return true;
}

void DJIR_SDK::DataHandle::run()
{
    std::vector<uint8_t> v1_pack_list;
    std::size_t pack_len = 0;
    int step = 0;
    USBCAN_SDK::CANConnection* pDev = static_cast<USBCAN_SDK::CANConnection*>(_dev);
    bool bFirstLoop = true;

    while (!_stopped.load())
    {
        // USBCAN_SDK now exposes only the queue-based receive path. This keeps
        // packet parsing on a single consumer path instead of the old queue/map
        // split model.
        auto frame = pDev->get_tunnel()->pop_data_from_recv_queue();
        for (std::size_t i = 0; i < frame.size(); ++i)
        {
            if (step == 0)
            {
                if (frame[i] == 0xAA)
                {
                    v1_pack_list.push_back(frame[i]);
                    step = 1;
                }
            }
            else if (step == 1)
            {
                pack_len = static_cast<std::size_t>(frame[i]);
                v1_pack_list.push_back(frame[i]);
                step = 2;
            }
            else if (step == 2)
            {
                pack_len |= (static_cast<std::size_t>(frame[i] & 0x3) << 8);
                v1_pack_list.push_back(frame[i]);
                step = 3;
            }
            else if (step == 3)
            {
                v1_pack_list.push_back(frame[i]);
                if (v1_pack_list.size() == 12)
                {
                    if (_check_head_crc(v1_pack_list))
                    {
                        step = 4;
                    }
                    else
                    {
                        step = 0;
                        v1_pack_list.clear();
                    }
                }
            }
            else if (step == 4)
            {
                v1_pack_list.push_back(frame[i]);
                if (v1_pack_list.size() == pack_len)
                {
                    step = 0;
                    if (_check_pack_crc(v1_pack_list))
                    {
                        // Hand off only fully assembled and CRC-validated packets.
                        _process_cmd(v1_pack_list);
                    }
                    v1_pack_list.clear();
                }
            }
            else
            {
                step = 0;
                v1_pack_list.clear();
            }
        }

        if (bFirstLoop)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(_DJIR_PKT_QUERY_FIRST_LOOP_MS));
            bFirstLoop = false;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(_DJIR_PKT_QUERY_INTERVAL_MS));
        }
    }
}

void DJIR_SDK::DataHandle::_process_cmd(const std::vector<uint8_t>& data)
{
    if (data.size() < 14)
    {
        return;
    }

    const uint8_t cmd_type = data[3];
    bool bOk = false;
    uint8_t cmd_key[2] = {0, 0};

    if (cmd_type == 0x20)
    {
        // Response frames are matched against the pending command deque by the
        // header CRC currently used by this SDK family as its best-effort key.
        std::lock_guard<std::mutex> lock(_rdcontent_lock);
        const uint16_t uiDataCrc = ReadLeU16(data, 8);
        for (auto it = _vecPendingCmds.begin(); it != _vecPendingCmds.end(); ++it)
        {
            if (it->vCmd.size() >= 14 && it->uiHeaderCrc == uiDataCrc)
            {
                cmd_key[0] = it->vCmd[12];
                cmd_key[1] = it->vCmd[13];
                _vecPendingCmds.erase(it);
                bOk = true;
                break;
            }
        }
    }
    else
    {
        cmd_key[0] = data[12];
        cmd_key[1] = data[13];
        bOk = true;
    }

    if (!bOk)
    {
        return;
    }

    const uint16_t uiCmdKey = static_cast<uint16_t>(cmd_key[0]) |
                              (static_cast<uint16_t>(cmd_key[1]) << 8);

    switch (uiCmdKey)
    {
    case 0x090E:
    {
        uint32_t nDeviceId = 0;
        uint32_t nVersion = 0;
        if (cmd_type == 0x20)
        {
            if (data.size() >= 27)
            {
                nDeviceId = ReadLeU32(data, 15);
                nVersion = ReadLeU32(data, 19);
            }
        }
        else if (data.size() >= 26)
        {
            nDeviceId = ReadLeU32(data, 14);
            nVersion = ReadLeU32(data, 18);
        }

        DeviceVersionCallback fnDeviceVersion;
        {
            std::lock_guard<std::mutex> lock(_callback_mutex);
            fnDeviceVersion = _device_version_callback;
        }
        if (fnDeviceVersion && (nDeviceId != 0 || nVersion != 0))
        {
            fnDeviceVersion(nDeviceId, nVersion);
        }
        break;
    }
    case 0x020E:
    {
        if (data.size() < 22)
        {
            break;
        }

        const int16_t nYaw = ReadLeI16(data, 16);
        const int16_t nRoll = ReadLeI16(data, 18);
        const int16_t nPitch = ReadLeI16(data, 20);

        {
            // Publish the new sample under the same mutex/condvar domain used
            // by get_position(), then notify outside the critical section.
            std::lock_guard<std::mutex> lock(_input_position_mutex);
            _stPositionSample.nYaw = nYaw;
            _stPositionSample.nRoll = nRoll;
            _stPositionSample.nPitch = nPitch;
            _stPositionSample.bReady = true;
        }
        _input_position_cond_var.notify_one();

        PositionUpdateCallback fnPositionUpdate;
        {
            // Take a callback snapshot so user code never runs under SDK mutexes.
            std::lock_guard<std::mutex> lock(_callback_mutex);
            fnPositionUpdate = _position_update_callback;
        }
        if (fnPositionUpdate)
        {
            fnPositionUpdate(nYaw, nRoll, nPitch);
        }
        break;
    }
    case 0x080E:
    {
        if (data.size() < 31)
        {
            break;
        }

        TPushData push;
        push.ctrl_byte = data[14];
        push.bAnglesValid = (push.ctrl_byte & 0x01) != 0;
        push.nYawAttitude = ReadLeI16(data, 15);
        push.nRollAttitude = ReadLeI16(data, 17);
        push.nPitchAttitude = ReadLeI16(data, 19);
        push.nYawJoint = ReadLeI16(data, 21);
        push.nRollJoint = ReadLeI16(data, 23);
        push.nPitchJoint = ReadLeI16(data, 25);

        PushDataCallback fnPushData;
        {
            // Same snapshot rule as position callbacks.
            std::lock_guard<std::mutex> lock(_callback_mutex);
            fnPushData = _push_data_callback;
        }
        if (fnPushData)
        {
            fnPushData(push);
        }
        break;
    }
    default:
        break;
    }
}

bool DJIR_SDK::DataHandle::_check_head_crc(const std::vector<uint8_t>& data)
{
    if (data.size() < 12)
    {
        return false;
    }

    crc16_t crc16 = crc16_init();
    crc16 = crc16_update(crc16, data.data(), 10);
    crc16 = crc16_finalize(crc16);

    const uint16_t recv_crc = ReadLeU16(data, data.size() - 2);
    return crc16 == recv_crc;
}

bool DJIR_SDK::DataHandle::_check_pack_crc(const std::vector<uint8_t>& data)
{
    if (data.size() < 4)
    {
        return false;
    }

    crc32_t crc32 = crc32_init();
    crc32 = crc32_update(crc32, data.data(), data.size() - 4);
    crc32 = crc32_finalize(crc32);

    const uint32_t recv_crc = ReadLeU32(data, data.size() - 4);
    return crc32 == recv_crc;
}

void DJIR_SDK::DataHandle::set_position_update_callback(PositionUpdateCallback callback)
{
    std::lock_guard<std::mutex> lock(_callback_mutex);
    _position_update_callback = callback;
}

void DJIR_SDK::DataHandle::set_push_callback(PushDataCallback callback)
{
    std::lock_guard<std::mutex> lock(_callback_mutex);
    _push_data_callback = callback;
}

void DJIR_SDK::DataHandle::set_device_version_callback(DeviceVersionCallback callback)
{
    std::lock_guard<std::mutex> lock(_callback_mutex);
    _device_version_callback = callback;
}
