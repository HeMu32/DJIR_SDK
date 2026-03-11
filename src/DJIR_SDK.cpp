#include "DJIR_SDK.h"
#include "Handle.h"
#include "CmdCombine.h"

#include "USBCAN_SDK.h"
using namespace USBCAN_SDK;


enum FLAG : uint8_t {
    BIT1 = 0x01,
    BIT2 = 0x02,
    BIT3 = 0x04,
    BIT4 = 0x08,
    BIT5 = 0x10,
    BIT6 = 0x20,
    BIT7 = 0x40
};

DJIR_SDK::DJIRonin::DJIRonin()
{
    _can_conn   = nullptr;  // must be nullptr before first connect()
    _pack_thread = nullptr; // must be nullptr before first connect()
    _position_ctrl_byte = 0;
    _speed_ctrl_byte  = 0;

    _position_ctrl_byte |= BIT1;    //MoveMode - ABSOLUTE_CONTROL
    _speed_ctrl_byte    |= BIT3;    //SpeedControl - DISABLED, FocalControl - DISABLED
    _cmd_cmb = new CmdCombine();
}

DJIR_SDK::DJIRonin::~DJIRonin()
{
    // disconnect() is idempotent and null-safe; handles DataHandle stop + CAN teardown.
    // Do NOT call ~CANConnection() directly here — disconnect() already does it,
    // and calling it again on a nullptr (after disconnect) would crash.
    disconnect();
    if (_cmd_cmb)
    {
        delete (CmdCombine*)_cmd_cmb;
        _cmd_cmb = nullptr;
    }
}

bool DJIR_SDK::DJIRonin::connect(int iDevIndex, int iCanIndex)
{
    int send_id = 0x223;    // CAN Tx No. for PC side
    int recv_id = 0x222;    // CAN Rx No. for PC side
    std::string stCANBoxType = "GC_USBCAN";

    // Connect to DJIR gimbal.
    // iDevIndex / iCanIndex 透传至 CANConnection(tunnel_id, can_index)。
    // 原实现硬编码为 (0, 0)；CANConnection 默认 can_index=1，此处修改为透传调用方参数。
    // WARNING: 非默认设备/通道路径未经实机验证，行为存在不确定性。
    _can_conn = new CANConnection(send_id, recv_id, stCANBoxType, USBCAN_SDK::TunnelType::USBCAN_II_TYPE, iDevIndex, iCanIndex);
    _pack_thread = new DataHandle(_can_conn);
    ((DataHandle*)_pack_thread)->start();

    // FRAGILE: 此处硬编码等待是为了让 DataHandle 线程及 CAN 驱动完成内部初始化。
    // 原值 500ms，已降低至 100ms；较短的等待与上层握手逻辑并不互斥，两者可共存。
    // 仍存在在低速主机或驱动响应慢时提前返回的风险，但上层握手会捕获此类情况。
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return ((CANConnection*)_can_conn)->get_connection_status();
}

bool DJIR_SDK::DJIRonin::disconnect()
{
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
        // 原设计：调用显式析构而非 delete（与 new CANConnection() 搭配属非标准用法，保持原行为）。
        // 置 nullptr 防止重复析构。
        ((CANConnection*)_can_conn)->~CANConnection();
        _can_conn = nullptr;
    }
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);
    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    return ret > 0;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;

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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);
    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
        return true;
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return ((DataHandle*)_pack_thread)->get_position(yaw, roll, pitch, 50);;      ////////////////////////////////////////////
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
    
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
    {
        return true;
    }
    else
        return false;
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
    ((DataHandle*)_pack_thread)->add_cmd(cmd);

    int ret = ((CANConnection*)_can_conn)->send_cmd(cmd);
    if (ret > 0)
        return true;
    else
        return false;
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
