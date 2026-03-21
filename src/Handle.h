#ifndef HANDLE_H
#define HANDLE_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#if (defined _WIN32 && defined RF62X_LIBRARY)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT
#endif

#define _DJIR_PKT_QUERY_INTERVAL_MS 5
#define _DJIR_PKT_QUERY_FIRST_LOOP_MS 100

namespace DJIR_SDK {

/// @brief 位置更新回调：来自 0x02 响应（姿态角，单位 0.1°）
typedef std::function<void(int16_t yaw, int16_t roll, int16_t pitch)> PositionUpdateCallback;

/**
 * @brief 0x08 参数推送数据（手持云台参数推送，协议 2.3.4.9）
 */
struct TPushData
{
    uint8_t  ctrl_byte       = 0;
    bool     bAnglesValid    = false;
    int16_t  nYawAttitude    = 0;
    int16_t  nRollAttitude   = 0;
    int16_t  nPitchAttitude  = 0;
    int16_t  nYawJoint       = 0;
    int16_t  nRollJoint      = 0;
    int16_t  nPitchJoint     = 0;
};

typedef std::function<void(const TPushData&)> PushDataCallback;
typedef std::function<void(uint32_t nDeviceId, uint32_t nVersion)> DeviceVersionCallback;

API_EXPORT class DataHandle
{
public:
    /// @brief Owns the SDK-side receive/parse worker for one CAN connection.
    ///        This object is not copyable and is expected to be 1:1 with a
    ///        single DJIRonin session.
    DataHandle(void* pCanConnection);
    ~DataHandle();

    /// @brief Start the packet parsing worker thread. Repeated start() calls are ignored.
    void start();

    /// @brief Stop the worker thread and join it. Safe to call multiple times.
    void stop();

    /// @brief Track a sent command that is expected to receive a response.
    ///        Matching is still header-CRC based, but the state now lives in a
    ///        mutex-protected pending-command deque instead of an unstructured vector.
    void add_cmd(std::vector<uint8_t> cmd);

    /// @brief Wait for the next 0x02 position sample.
    ///        The sample and ready flag are protected by a single mutex so the
    ///        condition_variable predicate and the returned data stay coherent.
    bool get_position(int16_t& yaw, int16_t& roll, int16_t& pitch, uint16_t timeout_ms);

    /// @brief Set 0x02 position callback. Runtime replacement is allowed.
    void set_position_update_callback(PositionUpdateCallback callback);

    /// @brief Set 0x08 push callback. Runtime replacement is allowed.
    void set_push_callback(PushDataCallback callback);

    /// @brief Set 0x09 device version callback. Runtime replacement is allowed.
    void set_device_version_callback(DeviceVersionCallback callback);

private:
    struct TPendingCmd
    {
        /// @brief Original command bytes kept for response CmdSet/CmdID recovery.
        std::vector<uint8_t> vCmd;

        /// @brief Header CRC used as the current best-effort response matching key.
        uint16_t uiHeaderCrc = 0;
    };

    struct TPositionSample
    {
        /// @brief True once a fresh 0x02 sample has been written and not yet consumed by get_position().
        bool bReady = false;
        int16_t nYaw = 0;
        int16_t nRoll = 0;
        int16_t nPitch = 0;
    };

private:
    /// @brief Main worker loop: pop raw CAN payload bytes from USBCAN_SDK,
    ///        assemble protocol packets, validate CRC, then dispatch parsed frames.
    void run();

    /// @brief Parse a validated full packet and dispatch response/push callbacks.
    void _process_cmd(const std::vector<uint8_t>& data);

    /// @brief Validate protocol head CRC16.
    bool _check_head_crc(const std::vector<uint8_t>& data);

    /// @brief Validate full packet CRC32.
    bool _check_pack_crc(const std::vector<uint8_t>& data);

private:
    std::thread _thread;

    /// @brief Stop flag shared with the worker thread.
    std::atomic<bool> _stopped {false};
    void* _dev = nullptr;

    /// @brief Protects pending-response command tracking.
    std::mutex _rdcontent_lock;

    /// @brief Commands awaiting response matching.
    std::deque<TPendingCmd> _vecPendingCmds;

    /// @brief Protects the blocking get_position() sample state.
    std::mutex _input_position_mutex;
    std::condition_variable _input_position_cond_var;
    TPositionSample _stPositionSample;

    /// @brief Protects runtime callback replacement against worker-thread reads.
    std::mutex _callback_mutex;
    PositionUpdateCallback _position_update_callback;
    PushDataCallback _push_data_callback;
    DeviceVersionCallback _device_version_callback;
};

}

#endif //HANDLE_H
