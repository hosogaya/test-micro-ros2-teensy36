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
    motors_in_serial4.attach(Serial4, 2e6);
    if (!motors_in_serial3.addModel(ids3, origin3)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);   
    }

    if (!motors_in_serial4.addModel(ids4, origin4)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);   
    }
    motors_in_serial3.setup(_torque);
    motors_in_serial4.setup(_torque);
}

void readMotorStates3() {
    threads.setSliceMicros(50);
    size_t t = millis();
    while (1) {
        t = millis();
        {
            Threads::Scope scope(mx_read3);
            if (!motors_in_serial3.readData()) {
                Serial.println("Serial3: Failed to read data");
                threads.delay(500);
            }
            else {
                Serial.println("Serial3: Success to read data");
                threads.delay(500);
            }
        }
        // {
        //     Threads::Scope scope(mx_write3);
        //     motors_in_serial3.writeData();
        // }
        while (millis() - t < 10) threads.yield();
    }
}

void readMotorStates4() {
    threads.setSliceMicros(50);
    size_t t = millis();
    while (1) {
        t = millis();
        {
            Threads::Scope scope(mx_read4);
            if (!motors_in_serial4.readData()) {
                Serial.println("Serial4: Failed to read data");
                threads.delay(500);
            };
        }
        // {
        //     Threads::Scope scope(mx_write4);
        //     motors_in_serial4.writeData();
        // }
        while (millis() - t < 10) threads.yield();
    }
}

// start threads
void start() {
    threads.addThread(readMotorStates3, 0, 4096);
    // threads.addThread(readMotorStates4, 0, 4096);
}

void setInputs(const std::array<float, 18>& _inputs) {
    size_t num3 = 0;
    {
        Threads::Scope scope(mx_write3);
        if (motors_in_serial3.getSize()!=motors_in_serial3.input_vels_.size())
            motors_in_serial3.input_vels_.resize(motors_in_serial3.getSize());        
        num3 = motors_in_serial3.getSize();
        for (size_t i=0; i<motors_in_serial3.getSize(); ++i) {
            motors_in_serial3.input_vels_[i] = _inputs[i];    
        }
    }
    {
        Threads::Scope scope(mx_write4);
        if (motors_in_serial4.getSize()!=motors_in_serial4.input_vels_.size())
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
            _errs[i+num3] = static_cast<float>(motors_in_serial4.getErrs()[i]);
        }
    }
}

Motors::Motors(const uint8_t _pin = 0) : DxlCtl(_pin) {}

void Motors::setup(const bool _torque) {
    addIndirectData(DxlCtl::Reg::HardwareErrorStatus, 1);
    addIndirectData(DxlCtl::Reg::PresentCurrent, 2);
    addIndirectData(DxlCtl::Reg::PresentVelocity, 4);
    addIndirectData(DxlCtl::Reg::PresentPosition, 4);

    syncWriteOperatingMode(1); // velocity control
    syncWriteVelocityPgain(400);
    syncWriteVelocityIgain(3840);

    if (_torque) syncEnableTorque();
}

bool Motors::readData() {
    return syncReadPosVelCurErr(errs_, rads_, vels_, curs_);
}

bool Motors::writeData() {
    return syncWriteVel(input_vels_);
}

void Motors::printStatus() const {
    Serial.print("size: "); Serial.println(getSize());
    Serial.print("IDs: ");
    for (size_t i=0; i<id_.size(); ++i) {
        Serial.print(this->id_[i]); Serial.print(" "); 
    }
    Serial.println();
    //
    Serial.print("Number of Indirect Data: "); Serial.println(indirect_data_num_);
    Serial.print("Size of Indirect Data: ");
    for (size_t i=0; i<indirect_data_size_.size(); ++i) {
        Serial.print(this->indirect_data_size_[i]); Serial.print(" "); 
    }
    Serial.println();
    //
}

void Motors::printMotorStates() const {
    Serial.print("Angle: ");
    for (size_t i=0; i<rads_.size(); ++i) {
        Serial.print(this->rads_[i], 2); Serial.print(" "); 
    }
    Serial.println();
    //
    Serial.print("Velocity: ");
    for (size_t i=0; i<vels_.size(); ++i) {
        Serial.print(this->vels_[i], 2); Serial.print(" "); 
    }
    Serial.println();
    //
    Serial.print("Current: ");
    for (size_t i=0; i<curs_.size(); ++i) {
        Serial.print(this->curs_[i], 2); Serial.print(" "); 
    }
    Serial.println();
    //
    Serial.print("Error: ");
    for (size_t i=0; i<errs_.size(); ++i) {
        Serial.print(this->errs_[i]); Serial.print(" "); 
    }
    Serial.println();
    //
}

void printStatus() {
    Serial.println("Serial 3");
    motors_in_serial3.printStatus();
    Serial.println("Serial 4");
    motors_in_serial4.printStatus();
}

void printMotorStates() {
    Serial.println("Serial 3");
    motors_in_serial3.printMotorStates();
    Serial.println("Serial 4");
    motors_in_serial4.printMotorStates();
}

void test_setup() {
    std::vector<uint8_t> ids = {31};
    std::vector<int32_t> origin = {2048};
    motors_in_serial4.attach(Serial4, 2e6);
    if (!motors_in_serial4.addModel(ids, origin)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);   
    }
    motors_in_serial4.setup(true);
    motors_in_serial4.input_vels_.resize(1);
    motors_in_serial4.input_vels_[0] = 1.0f;
}

void test_readMotorStates() {
     threads.setSliceMicros(20);
    size_t t = millis();
    while (1) {
        t = millis();
        {
            Threads::Scope scope(mx_read4);
            motors_in_serial4.readData();
        }
        {
            Threads::Scope scope(mx_write4);
            motors_in_serial4.writeData();
        }
        while (millis() - t < 10) threads.yield();
    }
}

void test_start() {
    threads.addThread(test_readMotorStates, 0, 2048);
}

void test_readJointState(std::array<float, 1>& _angles, std::array<float,1>& _vels, 
    std::array<float,1>& _curs, std::array<float, 1>& _errs)
{
    {
        Threads::Scope scope(mx_read4);
        if (motors_in_serial4.getSize() != 1) {
            Serial.println("Error");
            exit(1);
        }
        for (size_t i=0; i<motors_in_serial4.getSize(); ++i) {
            _angles[i] = motors_in_serial4.getRads()[i];
            _vels[i] = motors_in_serial4.getVels()[i];
            _curs[i] = motors_in_serial4.getCurs()[i];
            _errs[i] = static_cast<float>(motors_in_serial4.getErrs()[i]);
        }
    }
}


}