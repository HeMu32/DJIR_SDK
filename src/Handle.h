#ifndef HANDLE_H
#define HANDLE_H

#include <thread>
#include <chrono>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

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
    uint8_t  ctrl_byte       = 0;   ///< 有效标志字节（bit0=角度有效, bit1=限位有效, bit2=力度有效）
    bool     bAnglesValid    = false; ///< ctrl_byte[0]：姿态角/关节角是否有效
    int16_t  nYawAttitude    = 0;   ///< 姿态角 Yaw（0.1°）
    int16_t  nRollAttitude   = 0;   ///< 姿态角 Roll（0.1°）
    int16_t  nPitchAttitude  = 0;   ///< 姿态角 Pitch（0.1°）
    int16_t  nYawJoint       = 0;   ///< 关节角 Yaw（0.1°）
    int16_t  nRollJoint      = 0;   ///< 关节角 Roll（0.1°）
    int16_t  nPitchJoint     = 0;   ///< 关节角 Pitch（0.1°）
};

/// @brief 0x08 参数推送回调
typedef std::function<void(const TPushData&)> PushDataCallback;

/// @brief 0x09 设备版本回调（响应帧或设备主动推送, 1Hz）
typedef std::function<void(uint32_t nDeviceId, uint32_t nVersion)> DeviceVersionCallback;

API_EXPORT class DataHandle
{
public:
    DataHandle(void*);
    ~DataHandle();
    void start();
    void stop();

    void add_cmd(std::vector<uint8_t> cmd);

    bool get_position(int16_t& yaw, int16_t& roll, int16_t& pitch, uint16_t timeout_ms);

    /// @brief 设置 0x02 响应的位置更新回调
    void set_position_update_callback(PositionUpdateCallback callback);

    /// @brief 设置 0x08 参数推送回调
    void set_push_callback(PushDataCallback callback);

    /// @brief 设置 0x09 设备版本信息回调
    void set_device_version_callback(DeviceVersionCallback callback);

private:
    void run();
    void _process_cmd(std::vector<uint8_t> data);
    bool _check_head_crc(std::vector<uint8_t> data);
    bool _check_pack_crc(std::vector<uint8_t> data);
    std::thread _thread;
    bool _stopped = false;
    void* _dev;
    std::vector<std::string> _rsps;
    std::mutex _rdcontent_lock;

    // get_position fallback (blocking)
    std::mutex _input_position_mutex;
    std::condition_variable _input_position_cond_var;
    bool _input_position_ready_flag;
    int16_t _yaw; int16_t _roll; int16_t _pitch;

    PositionUpdateCallback _position_update_callback;
    PushDataCallback       _push_data_callback;
    DeviceVersionCallback  _device_version_callback;

    std::vector<std::vector<uint8_t>> _cmd_list;
};

}

#endif //HANDLE_H
