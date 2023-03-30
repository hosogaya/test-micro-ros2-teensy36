#include <Arduino.h>
#include <TeensyThreads.h>
#include <motor_control.h>
#include <array>

namespace motor_control {
Motors motors_in_serial3;
Motors motors_in_serial4;

Threads::Mutex mx_read3, mx_write3;
Threads::Mutex mx_read4, mx_write4;

void setup(const bool _torque) {
    std::vector<uint8_t> ids3 = {1,2,3,11,12,13,21,22,23};
    std::vector<uint8_t> ids4 = {31,32,33,41,42,43,51,52,53};
    std::vector<int32_t> origin3 = {2048, 995, 1405, 2048, 995, 1405, 2048, 995, 1405};
    std::vector<int32_t> origin4 = {2048, 995, 1405, 2048, 995, 1405, 2048, 995, 1405};
    motors_in_serial3.attach(Serial3, 2e6);
    if (!motors_in_serial3.addModel(ids3, origin3)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);   
    }
    motors_in_serial3.setup(_torque);

    motors_in_serial4.attach(Serial4, 2e6);
    if (!motors_in_serial4.addModel(ids4, origin4)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);   
    }
    motors_in_serial3.setup(_torque);
    motors_in_serial4.setup(_torque);
}

void readMotorStates3() {
    threads.setSliceMicros(20);
    size_t t = millis();
    while (1) {
        t = millis();
        // {
        //     Threads::Scope scope(mx_read3);
        //     motors_in_serial3.readData();
        // }
        // {
        //     Threads::Scope scope(mx_write3);
        //     motors_in_serial3.writeData();
        // }
        while (millis() - t < 10) threads.yield();
    }
}

void readMotorStates4() {
    threads.setSliceMicros(20);
    size_t t = millis();
    while (1) {
        t = millis();
        // {
        //     Threads::Scope scope(mx_read4);
        //     motors_in_serial4.readData();
        // }
        // {
        //     Threads::Scope scope(mx_write4);
        //     motors_in_serial4.writeData();
        // }
        while (millis() - t < 10) threads.yield();
    }
}

// start threads
void start() {
    threads.addThread(readMotorStates3, 0, 2048);
    threads.addThread(readMotorStates4, 0, 2048);
}

void setInputs(const std::array<float, 18>& _inputs) {
    size_t num3 = 0;
    {
        Threads::Scope scope(mx_write3);
        motors_in_serial3.input_vels_.resize(motors_in_serial3.getSize());        
        num3 = motors_in_serial3.getSize();
        for (size_t i=0; i<motors_in_serial3.getSize(); ++i) {
            motors_in_serial3.input_vels_[i] = _inputs[i];    
        }
    }
    {
        Threads::Scope scope(mx_write4);
        motors_in_serial4.input_vels_.resize(motors_in_serial4.getSize());        
        for (size_t i=0; i<motors_in_serial4.getSize(); ++i) {
            motors_in_serial4.input_vels_[i] = _inputs[i+num3];
        }
    }
}


void readJointState(std::array<float, 18>& _angles, std::array<float,18>& _vels, 
    std::array<float,18>& _curs, std::array<float, 18>& _errs) 
{
    size_t num3 = 0;
    {
        Threads::Scope scope(mx_read3);
        num3 = motors_in_serial3.getSize();
        for (size_t i=0; i<motors_in_serial3.getSize(); ++i) {
            _angles[i] = motors_in_serial3.getRads()[i];
            _vels[i] = motors_in_serial3.getVels()[i];
            _curs[i] = motors_in_serial3.getCurs()[i];
            _errs[i] = motors_in_serial3.getErrs()[i];
        }
    }
    {
        Threads::Scope scope(mx_read4);
        for (size_t i=0; i<motors_in_serial4.getSize(); ++i) {
            _angles[i+num3] = motors_in_serial4.getRads()[i];
            _vels[i+num3] = motors_in_serial4.getVels()[i];
            _curs[i+num3] = motors_in_serial4.getCurs()[i];
            _errs[i+num3] = motors_in_serial4.getErrs()[i];
        }
    }
}

Motors::Motors(const uint8_t _pin = 0) : DxlCtl(_pin) {}

void Motors::setup(const bool _torque) {
    addIndirectData(DxlCtl::Reg::TorqueEnable, 1);
    addIndirectData(DxlCtl::Reg::HardwareErrorStatus, 1);
    addIndirectData(DxlCtl::Reg::PresentCurrent, 2);
    addIndirectData(DxlCtl::Reg::PresentVelocity, 4);
    addIndirectData(DxlCtl::Reg::PresentPosition, 4);

    syncWriteOperatingMode(1); // velocity control
    syncWriteVelocityPgain(400);
    syncWriteVelocityIgain(3840);
    if (_torque) syncEnableTorque();
}

void Motors::readData() {
    syncReadPosVelCurErr(errs_, rads_, vels_, curs_);
}

void Motors::writeData() {
    syncWriteVel(input_vels_);
}

}