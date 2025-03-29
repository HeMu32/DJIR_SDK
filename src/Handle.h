#ifndef HANDLE_H
#define HANDLE_H

#include <thread>
#include <chrono>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>  // 添加这行，用于支持回调函数

#if (defined _WIN32 && defined RF62X_LIBRARY)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT
#endif

namespace DJIR_SDK {

// 定义位置更新回调函数类型
typedef std::function<void(int16_t yaw, int16_t roll, int16_t pitch)> PositionUpdateCallback;

API_EXPORT class DataHandle
{
public:
    DataHandle(void*);
    ~DataHandle();
    void start();
    void stop();

    void add_cmd(std::vector<uint8_t> cmd);

    bool get_position(int16_t& yaw, int16_t& roll, int16_t& pitch, uint16_t timeout_ms);
    
    // 设置位置更新回调函数
    void set_position_update_callback(PositionUpdateCallback callback);

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

    // get_position
    std::mutex _input_position_mutex;
    std::condition_variable _input_position_cond_var;
    bool _input_position_ready_flag;
    int16_t _yaw; int16_t _roll; int16_t _pitch;

    // 位置更新回调函数
    PositionUpdateCallback _position_update_callback;

    std::vector<std::vector<uint8_t>> _cmd_list;
};

}

#endif //HANDLE
