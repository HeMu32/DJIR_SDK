#ifndef DJI_R_SDK_H
#define DJI_R_SDK_H

#include <stdlib.h>
#include <stdint.h>
#include <vector>

#if (defined _WIN32 && defined RF62X_LIBRARY)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT
#endif

namespace DJIR_SDK {

enum class ReturnCode {
    EXECUTION_SUCCESSFUL = 0,
    PARSE_ERROR = 1,
    EXECUTION_FAILS = 2
};

enum class AxisType {
    YAW = 0,
    ROLL = 1,
    PITCH = 2
};

enum class MoveMode {
    INCREMENTAL_CONTROL = 0,
    ABSOLUTE_CONTROL = 1
};

enum class SpeedControl {
    DISABLED = 0,
    ENABLED = 1
};

enum class FocalControl {
    ENABLED = 0,
    DISABLED = 1
};



API_EXPORT class DJIRonin
{
public:
    DJIRonin();
    ~DJIRonin();

    /**
     * @brief               connect - Connect to DJI Ronin device
     * @param iDevIndex     Index of CAN box device on this PC system, starting from 0
     * @param iCanIndex     Index of CAN channel on the CAN device, strating from 0
     * @return              True if success
     */
    bool connect(int iDevIndex, int iCanIndex);
    /**
     * @brief disconnect - Disconnect from DJI Ronin device
     * @return True if success
     */
    bool disconnect();

    /**
     * @brief           move_to - Handheld Gimbal Position Control (p.5, 2.3.4.1)
     * @param iYaw      Yaw angle, unit: 0.1°   (range: -1800 to +1800)
     * @param iRoll     Roll angle, unit: 0.1°  (range: -300  to +300)
     * @param iPitch    Pitch angle, unit: 0.1° (range: -560  to +1460)
     * @param time_ms   Command execution speed, unit: ms. Min value is 100ms.
     *                  Time is used to set motion speed when gimbal is executing this command.
     * @return          True if success
     */
    bool move_to(int16_t iYaw, int16_t iRoll, int16_t iPitch, uint16_t time_ms);

    /**
     * @brief set_inverted_axis - Handheld Gimbal Position Control (p.5, 2.3.4.1)
     * @param axis Type of axis (YAW, ROLL, PITCH)
     * @param invert True if invert
     * @return True if success
     */
    bool set_inverted_axis(AxisType axis, bool invert);

    /**
     * @brief set_move_mode Set move mode
     * @param type INCREMENTAL or ABSOLUTE
     * @return True if success
     */
    bool set_move_mode(MoveMode type);

    /**
     * @brief           set_speed - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @details         This command can only control for 0.5s each time it is issued
     *                  due to safety reasons. If users require continuous speed, they can send
     *                  this command periodically. If users want to stop the rotation of three
     *                  axes immediately, they can set the fields of iYaw, iPitch, and iRoll in
     *                  set_speed_mode method as 0.
     * @param iYaw      Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @param iRoll     Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @param iPitch    Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @return          True if success
     */
    bool set_speed(uint16_t iYaw, uint16_t iRoll, uint16_t iPitch);

    /**
     * @brief               set_speed_mode - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @param speed_type    Enable or Disable speed control
     * @param focal_type    Enable or Disable the impact of camera focal length
     *                      into consideration
     * @return              True if success
     */
    bool set_speed_mode(SpeedControl speed_type, FocalControl focal_type);


    /**
     * @brief           get_current_position - Get Gimbal Information (p.6, 2.3.4.3)
     * @param iYaw      Yaw angle, unit: 0.1° (range: -1800 to +1800)
     * @param iRoll     Roll angle, unit: 0.1° (range: -1800 to +1800)
     * @param iPitch    Pitch angle, unit: 0.1° (range: -1800 to +1800)
     * @return          True if success
     */
    bool get_current_position(int16_t &iYaw, int16_t &iRoll, int16_t &iPitch);
    

    /// @brief  Not working, may try setpos(0,0,0) instead
    /// @param  
    /// @return 
    bool recenter (void);


    /// @brief          Set absolute position value in calibrated limitation range for focus motor. 
    ///                 Expecting 100Hz pushing rate, timing mechanism implemented by caller.
    /// @param uiPos    Adsolute position, 0~4095
    /// @return         True if success
    bool set_focus_motor_pos (uint16_t uiPos);


    /// @brief          Get current position in calibrated limitation range of focus motor.
    ///                 Not working with RS 2. No alternative. May avoid to use.
    ///                 Expect data handler (Handel.cpp/_process_cmd()) to deliver proper msg to message tunnel.
    /// @param flagStat Calibration status, 0 - not calibrated, 1 - auto cal in progress, 2 - finished.
    /// @param iPos     Absolute position the motor is in the limitation range, 0~4095.
    /// @return         True if success.
    bool get_focus_motor_pos (void);


    /// @brief  Command the gimbal to start focus motor auto calibration for limitation
    /// @param  
    /// @return True if success
    bool start_focus_motor_auto_cal (void);


    /// @brief      Push joystick position to gimbal to controll movement.
    ///             Expected to be called with an interval, or it exaust the system.
    ///             Interval of 50ms has been proven to work.
    /// @param iX   x position (horizontal), -15,000 ~ 15,000, mapped to pitch.
    /// @param iY   y position (vertical),   -15,000 ~ 15,000, mapped to yaw.
    /// @return     True if success
    bool push_joystick_pos_movement (uint16_t iX, uint16_t iY);


private:
    void* _can_conn;
    void* _pack_thread;
    uint8_t _position_ctrl_byte;
    uint8_t _speed_ctrl_byte;
    void* _cmd_cmb;
};


}


#endif //DJI_R_SDK_H
