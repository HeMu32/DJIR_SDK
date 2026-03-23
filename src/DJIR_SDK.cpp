#include "DJIR_SDK.h"
#include "Handle.h"
#include "CmdCombine.h"

#include "USBCAN_SDK.h"
using namespace USBCAN_SDK;

#include <iostream>
#include <thread>


enum FLAG : uint8_t {
    BIT1 = 0x01,
    BIT2 = 0x02,
    BIT3 = 0x04,
    BIT4 = 0x08,
    BIT5 = 0x10,
    BIT6 = 0x20,
    BIT7 = 0x40
};

namespace
{
constexpr uint16_t kGetCurrentPositionTimeoutMs = 500;

void LogDJIRoninLifecycle(const char* pszStage, const DJIR_SDK::DJIRonin* pSelf)
{
#if defined(_DEBUG)
    std::cerr << "[Lifecycle][DJIR_SDK::DJIRonin] " << pszStage
              << " this=" << pSelf
              << " thread=" << std::this_thread::get_id()
              << std::endl;
#else
    (void)pszStage;
    (void)pSelf;
#endif
}

bool EnqueueAndSendCmd(
    void* pPackThread,
    void* pCanConn,
    const std::vector<uint8_t>& cmd,
    bool bTrackResponse)
{
    if (pPackThread == nullptr || pCanConn == nullptr)
    {
        return false;
    }

    DJIR_SDK::DataHandle* pHandle = static_cast<DJIR_SDK::DataHandle*>(pPackThread);
    CANConnection* pConn = static_cast<CANConnection*>(pCanConn);
    if (bTrackResponse)
    {
        pHandle->add_cmd(cmd);
    }

    return pConn->send_cmd(cmd) > 0;
}
}

DJIR_SDK::DJIRonin::DJIRonin()
{
    LogDJIRoninLifecycle("ctor begin", this);
    _can_conn   = nullptr;  // must be nullptr before first connect()
    _pack_thread = nullptr; // must be nullptr before first connect()
    _position_ctrl_byte = 0;
    _speed_ctrl_byte  = 0;

    _position_ctrl_byte |= BIT1;    //MoveMode - ABSOLUTE_CONTROL
    _speed_ctrl_byte    |= BIT3;    //SpeedControl - DISABLED, FocalControl - DISABLED
    _cmd_cmb = new CmdCombine();
    LogDJIRoninLifecycle("ctor end", this);
}

DJIR_SDK::DJIRonin::~DJIRonin()
{
    LogDJIRoninLifecycle("dtor begin", this);
    // disconnect() is idempotent and null-safe; handles DataHandle stop + CAN teardown.
    // Do NOT call ~CANConnection() directly here — disconnect() already does it,
    // and calling it again on a nullptr (after disconnect) would crash.
    disconnect();
    if (_cmd_cmb)
    {
        delete (CmdCombine*)_cmd_cmb;
        _cmd_cmb = nullptr;
    }
    LogDJIRoninLifecycle("dtor end", this);
}

bool DJIR_SDK::DJIRonin::connect(int iDevIndex, int iCanIndex)
{
    LogDJIRoninLifecycle("connect begin", this);
    int send_id = 0x223;    // CAN Tx No. for PC side
    int recv_id = 0x222;    // CAN Rx No. for PC side
    std::string stCANBoxType = "GC_USBCAN";

    // reconnect 前先清理旧会话，避免失败重试路径叠加残留线程/句柄。
    disconnect();

    // Connect to DJIR gimbal.
    // iDevIndex / iCanIndex 透传至 CANConnection(tunnel_id, can_index)。
    // 原实现硬编码为 (0, 0)；CANConnection 默认 can_index=1，此处修改为透传调用方参数。
    // WARNING: 非默认设备/通道路径未经实机验证，行为存在不确定性。
    _can_conn = new CANConnection(send_id, recv_id, stCANBoxType, USBCAN_SDK::TunnelType::USBCAN_II_TYPE, iDevIndex, iCanIndex);
    _pack_thread = new DataHandle(_can_conn);
    ((DataHandle*)_pack_thread)->start();

    // FRAGILE: 此处硬编码等待是为了让 DataHandle 线程及 CAN 驱动完成内部初始化。
    // 原值 500ms，已降低至 1000ms；较短的等待与上层握手逻辑并不互斥，两者可共存。
    // 仍存在在低速主机或驱动响应慢时提前返回的风险，但上层握手会捕获此类情况。
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    const bool bRet = ((CANConnection*)_can_conn)->get_connection_status();
    LogDJIRoninLifecycle("connect end", this);
    return bRet;
}

bool DJIR_SDK::DJIRonin::disconnect()
{
    LogDJIRoninLifecycle("disconnect begin", this);
    // 先停止 DataHandle 线程（设 _stopped=true 并 join），再关闭 CAN 连接，
    // 避免 CAN 关闭后 DataHandle 仍在访问已释放资源。
    // DataHandle::run() 每次循环最多睡眠 100ms，join 最迟在此之内完成。
    if (_pack_thread)
    {
        ((DataHandle*)_pack_thread)->stop();
        delete (DataHandle*)_pack_thread;
        _pack_thread = nullptr;
    }
    if (_can_conn)
    {
        delete (CANConnection*)_can_conn;
        _can_conn = nullptr;
    }
    LogDJIRoninLifecycle("disconnect end", this);
    return true;
}

bool DJIR_SDK::DJIRonin::enable_push()
{
    if (!_pack_thread || !_can_conn)
        return false;

    // 协议 2.3.4.8：CmdSet=0x0E CmdID=0x07，ctrl_byte=0x01 使能参数推送
    uint8_t cmd_type = 0x03;  // fire-and-forget（与官方示例软件一致）
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x07;

    std::vector<uint8_t> data_payload = { 0x01 };  // 0x01 = enable push

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::set_push_callback(
    std::function<void(uint8_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t)> callback)
{
    if (!_pack_thread)
        return false;

    // 将扁平参数签名适配为 Handle 内部的 TPushData 回调
    ((DataHandle*)_pack_thread)->set_push_callback(
        [callback](const DJIR_SDK::TPushData& push)
        {
            callback(push.ctrl_byte,
                     push.nYawAttitude,  push.nRollAttitude,  push.nPitchAttitude,
                     push.nYawJoint,     push.nRollJoint,     push.nPitchJoint);
        });
    return true;
}

bool DJIR_SDK::DJIRonin::set_device_version_callback(
    std::function<void(uint32_t, uint32_t)> callback)
{
    if (!_pack_thread)
        return false;
    ((DataHandle*)_pack_thread)->set_device_version_callback(callback);
    return true;
}

bool DJIR_SDK::DJIRonin::request_device_version (void)
{
    uint8_t cmd_type = 0x02;    // response in need
    uint8_t cmd_set  = 0x0E;    
    uint8_t cmd_id   = 0x09;    // get version number

    std::vector<uint8_t> data_payload =
    {
        0x01,
        0x00,
        0x00,
        0x00    // device ID: 0x00000001: DJI R SDK
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);

}

bool DJIR_SDK::DJIRonin::move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x00;
    uint8_t time     = (uint8_t)(time_ms/100);

    std::vector<uint8_t> data_payload =
    {
        ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
        ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
        ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
        _position_ctrl_byte, time
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::set_inverted_axis(DJIR_SDK::AxisType axis, bool invert)
{
    if (axis == AxisType::YAW)
    {
        if (invert)
            _position_ctrl_byte |= BIT2;
        else
            _position_ctrl_byte &= ~BIT2;
    }

    if (axis == AxisType::ROLL)
    {
        if (invert)
            _position_ctrl_byte |= BIT3;
        else
            _position_ctrl_byte &= ~BIT3;
    }

    if (axis == AxisType::PITCH)
    {
        if (invert)
            _position_ctrl_byte |= BIT4;
        else
            _position_ctrl_byte &= ~BIT4;
    }

    return true;
}

bool DJIR_SDK::DJIRonin::set_move_mode(DJIR_SDK::MoveMode type)
{
    if (type == MoveMode::INCREMENTAL_CONTROL)
    {
        _position_ctrl_byte &= ~BIT1;
    }
    else if (type == MoveMode::ABSOLUTE_CONTROL)
    {
        _position_ctrl_byte |= BIT1;
    }
    else
    {
        return false;
    }

    return true;
}

bool DJIR_SDK::DJIRonin::set_speed(uint16_t yaw, uint16_t roll, uint16_t pitch)
{
    if (yaw > 3600)     yaw     = 3600;
    if (roll > 3600)    roll    = 3600;
    if (pitch > 3600)   pitch   = 3600;

    uint8_t cmd_type = 0x03;
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x01;

    std::vector<uint8_t> data_payload =
    {
        ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
        ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
        ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
        _speed_ctrl_byte
    };
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::set_speed_mode(DJIR_SDK::SpeedControl speed_type, DJIR_SDK::FocalControl focal_type)
{
    if (speed_type == SpeedControl::DISABLED)
    {
        _speed_ctrl_byte &= ~BIT7;
    }else
        _speed_ctrl_byte |= BIT7;

    if (focal_type == FocalControl::ENABLED)
    {
        _speed_ctrl_byte &= ~BIT3;
    }else
        _speed_ctrl_byte |= BIT3;

    return true;
}

bool DJIR_SDK::DJIRonin::get_current_position(int16_t &yaw, int16_t &roll, int16_t &pitch)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x02;

    std::vector<uint8_t> data_payload =
    {
        0x01
    };
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    const bool bSent = EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
    if (bSent)
    {
        return ((DataHandle*)_pack_thread)->get_position(yaw, roll, pitch, kGetCurrentPositionTimeoutMs);
    }
    else
        return false;
}

bool DJIR_SDK::DJIRonin::query_current_position()
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x02;

    std::vector<uint8_t> data_payload =
    {
        0x01
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::recenter (void)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x0E;

    std::vector<uint8_t> data_payload =
    {
        0xFE,
        0x0E
    };
    
    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::set_focus_motor_pos (uint16_t uiPos)
{
    // Structure copied from recenter() function
    uint8_t cmd_type = 0x00;    // No response in need
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x12;    // Focus motor
    uint8_t u8PosLo  = (uint8_t)(uiPos);
    uint8_t u8PosHi  = (uint8_t)(uiPos>>8);

    std::vector<uint8_t> data_payload =
    {
        0x01,       // Foucs motor control
        0x00,       // Focus control
        0x02,       // Data length (2 bytes), defined by SDK protocol
        u8PosLo,       // Convert to little endian
        u8PosHi
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::get_focus_motor_pos (void)
{
    uint8_t cmd_type = 0x02;    // response in need
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x12;    // focus motor

    std::vector<uint8_t> data_payload
    {
        0x15,       // Position poll
        0x00        // RS focus motor
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::start_focus_motor_auto_cal (void)
{
    // Structure copied from recenter() function
    uint8_t cmd_type = 0x03;    // response in need
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x12;    // focus motor

    std::vector<uint8_t> data_payload =
    {
        0x02,       // Foucs motor calibration
        0x00,       // RS focus motor
        0x01        // Auto calibration
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

bool DJIR_SDK::DJIRonin::push_joystick_pos_movement (int16_t iX, int16_t iY)
{
    iX = -iX;
    iY = -iY;

    if (iX > 15000) iX = 15000;
    if (iY > 15000) iY = 15000;
    if (iX < -15000) iX = -15000;
    if (iY < -15000) iY = -15000;


    uint8_t cmd_type = 0x00;    // no response in need
    uint8_t cmd_set  = 0x0E;    
    uint8_t cmd_id   = 0x0A;    // param push from external device

    std::vector<uint8_t> data_payload =
    {
        0x01,   // joystick controller
        (uint8_t)(iY),
        (uint8_t)(iY>>8),   // map Y pos to vertical movement
        0x00,
        0x00,               // leave roll parameter blank
        (uint8_t)(iX),
        (uint8_t)(iX>>8)    // map X pos to horizontal movement
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, false);
}

bool DJIR_SDK::DJIRonin::set_angle_limits(
    uint8_t pitch_max, uint8_t pitch_min,
    uint8_t yaw_max,   uint8_t yaw_min,
    uint8_t roll_max,  uint8_t roll_min)
{
    uint8_t cmd_type = 0x01;    // response preferred but not mandatory
    uint8_t cmd_set  = 0x0E;
    uint8_t cmd_id   = 0x03;    // set angle limits (2.3.4.4)

    std::vector<uint8_t> data_payload =
    {
        0x01,       // ctrl_byte: set angle limits
        pitch_max,
        pitch_min,
        yaw_max,
        yaw_min,
        roll_max,
        roll_min
    };

    auto cmd = ((CmdCombine*)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);
    return EnqueueAndSendCmd(_pack_thread, _can_conn, cmd, true);
}

// 实现 set_position_update_callback 方法
bool DJIR_SDK::DJIRonin::set_position_update_callback(std::function<void(int16_t, int16_t, int16_t)> callback)
{
    if (_pack_thread) {
        ((DataHandle*)_pack_thread)->set_position_update_callback(callback);
        return true;
    }
    return false;
}
