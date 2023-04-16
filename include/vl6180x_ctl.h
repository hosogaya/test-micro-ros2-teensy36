#pragma once
#include <VL6180X.h>
#include <PINs.h>
#include <vector>


namespace vl6180x_ctl {

class MultiVL6180X {
private:
    std::vector<VL6180X> devs_;
    std::vector<float> offsets_;
    std::vector<float> dists_;
    uint16_t period_ms_ = 50;
    size_t latest_read_time_ = 0;
    bool stable_ = false;
public:
    bool setup(TwoWire& s, const std::vector<uint8_t>& gpio, const std::vector<uint8_t>& addrs);
    bool setTimeout(const size_t _timeout); 
    bool setOffset(const std::vector<float>& _offsets);
    bool start(const uint16_t _period);
    bool read();
    const bool isStable() const {return stable_;}
    const std::vector<float> getDists() const {return dists_;}
    const float getPeriodMs() const {return period_ms_;}
    void printStatus() const;
};

void setup();
void start(); // add threads
void readVL6180XStates();
bool readDists(std::vector<float>& _dists);
void printDists();

}