#include <jy901_ctl.h>

namespace jy901_ctl {

IMU imu;
Threads::Mutex mx_jy901;

void setup() 
{
    imu.setup(230400, Serial5);
}

void readJY901Status()
{
    threads.setSliceMicros(50);
    size_t t = 0;
    while (1) {
        t = millis();
        {
            Threads::Scope scope(mx_jy901);
            imu.sensing();
        }
        while (millis() - t < 5) threads.yield();
    }
}

bool readData(std::array<float,3>& _pos, std::array<float,3>& _pos_vel, std::array<float,3>& _acc)
{
    bool result = false;
    {
        Threads::Scope scope(mx_jy901);
        _pos = imu.pos_;
        _pos_vel = imu.pos_vel_;       
        _acc = imu.acc_; 
        result = !imu.is_unsafe_;
    }
    return result;
}

void start() 
{
    threads.addThread(readJY901Status, 0, 1024);
}

}