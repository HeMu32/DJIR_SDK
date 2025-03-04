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
    _position_ctrl_byte = 0;
    _speed_ctrl_byte  = 0;

    _position_ctrl_byte |= BIT1;    //MoveMode - ABSOLUTE_CONTROL
    _speed_ctrl_byte |= BIT3;       //SpeedControl - DISABLED, FocalControl - DISABLED
    _cmd_cmb = new CmdCombine();
}

DJIR_SDK::DJIRonin::~DJIRonin()
{
    ((CANConnection*)_can_conn)->~CANConnection();
}

bool DJIR_SDK::DJIRonin::connect(int iDevIndex, int iCanIndex)
{
    int send_id = 0x223;    // CAN Tx No for PC side
    int recv_id = 0x222;    // CAN Rx No for PC side
    std::string stCANBoxType = "GC_USBCAN";

    // Connect to DJIR gimbal
    _can_conn = new CANConnection(send_id, recv_id, stCANBoxType, USBCAN_SDK::TunnelType::USBCAN_II_TYPE, 0, 0);
    _pack_thread = new DataHandle(_can_conn);
    ((DataHandle*)_pack_thread)->start();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return ((CANConnection*)_can_conn)->get_connection_status();
}

bool DJIR_SDK::DJIRonin::disconnect()
{
    ((CANConnection*)_can_conn)->~CANConnection();
    return true;
}

bool DJIR_SDK::DJIRonin::move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms)
{
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x00;
    uint8_t time = (uint8_t)(time_ms/100);
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
        return ((DataHandle*)_pack_thread)->get_position(yaw, roll, pitch, 1000);;
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

bool DJIR_SDK::DJIRonin::push_joystick_pos_movement (uint16_t iX, uint16_t iY)
{
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