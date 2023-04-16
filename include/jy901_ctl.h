#pragma once
#include <array>
#include <imu.h>
#include <TeensyThreads.h>

namespace jy901_ctl {

void setup();
void start();
void readJY901Status();
bool readData(std::array<float,3>& _pos, std::array<float,3>& _pos_vel, std::array<float,3>& _acc);
}