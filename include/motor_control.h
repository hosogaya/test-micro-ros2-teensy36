#pragma once
#include <dxl_ctl.h>
#include <cmath>

namespace motor_control {

class Motors : public DxlCtl {
    private:
        std::vector<float> rads_;
        std::vector<float> vels_;
        std::vector<float> curs_;
        std::vector<uint8_t> errs_;
        //
    public:
        Motors(const uint8_t _pin = 0);
        void setup(const bool _torque = false);
        void readData();
        void writeData();
        const std::vector<float>& getRads() const {return rads_;}
        const std::vector<float>& getVels() const {return vels_;}
        const std::vector<float>& getCurs() const {return curs_;}
        const std::vector<uint8_t>& getErrs() const {return errs_;}
        std::vector<float> input_vels_;
};


// called from main program
void setup(const bool _torque = false);
void start();
void setInputs(const std::array<float, 18>& _inputs);
void readJointState(std::array<float, 18>& _angles, std::array<float,18>& _vels, 
    std::array<float,18>& _curs, std::array<float, 18>& _errs);

}