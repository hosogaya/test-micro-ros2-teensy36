#include <vl6180x_ctl.h>
#include <TeensyThreads.h>

namespace vl6180x_ctl {

bool MultiVL6180X::setup(TwoWire& _s, const std::vector<uint8_t>& _gpio, const std::vector<uint8_t>& _addrs) 
{
    if (_gpio.size() != _addrs.size()) return false;
    devs_.resize(_gpio.size());
    offsets_.resize(_gpio.size());
    for (uint8_t i=0; i<_gpio.size(); ++i) {
        pinMode(_gpio[i], OUTPUT); delay(20);
        digitalWrite(_gpio[i], HIGH); delay(20);
        devs_[i].setAddress(_addrs[i]);
        devs_[i].init(); delay(20);
        offsets_[i] = 0.0f;
    }
    return true;
}

bool MultiVL6180X::setTimeout(const size_t _timeout) 
{
    if (devs_.size() <= 0) return false;
    for (VL6180X& dev: devs_) dev.setTimeout(_timeout);
    return true;
}

bool MultiVL6180X::setOffset(const std::vector<float>& _offsets)
{
    if (devs_.size() != _offsets.size()) return false;
    offsets_ = _offsets;
    return true;
}

bool MultiVL6180X::start(const uint16_t _period) 
{
    if (devs_.size() <= 0) return false;
    period_ms_ = _period;
    for (VL6180X& dev: devs_) 
    {
        uint16_t convergence_time = (float)_period * 0.9f - 4;
        dev.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, convergence_time);
        dev.stopContinuous(); delay(10);
        dev.startRangeContinuous(_period);
    }
    return true;
}

bool MultiVL6180X::read()
{
    if (devs_.size() != dists_.size()) dists_.resize(devs_.size());
    if (millis() - latest_read_time_ < period_ms_) return false;
    for (size_t i=0; i<devs_.size(); ++i)
    {
        dists_[i] = devs_[i].readRangeContinuousMillimeters() - offsets_[i];
        dists_[i] = std::min(255.0f, std::max(0.0f, dists_[i]));
        if (devs_[i].timeoutOccurred()) 
        {
            stable_ = false;
            return false;
        }
    }
    latest_read_time_ = millis();
    stable_ = true;
    return true;
}

void MultiVL6180X::printStatus() const 
{
    Serial.print("Dists: ");
    for (size_t i=0; i<devs_.size(); ++i)
    {
        Serial.print(dists_[i]); Serial.print(", ");
    }
}

MultiVL6180X vl6180x_in_wire;
Threads::Mutex mx;

void setup() 
{
    std::vector<uint8_t> cn = {CN19, CN18, CN17, CN16, CN15, CN14};
    std::vector<uint8_t> addrs = {0x30, 0x31, 0x22, 0x35, 0x24, 0x25};
    std::vector<float> offsets = {20, 18, 20, 21, 15, 20};
    for (size_t i=0; cn.size(); ++i) 
    {
        pinMode(cn[i], OUTPUT); delay(1);
        digitalWrite(cn[i], LOW); delay(1);
    }
    delay(200);
    vl6180x_in_wire.setup(Wire, cn, addrs);
    vl6180x_in_wire.setOffset(offsets);
    vl6180x_in_wire.setTimeout(2);
    vl6180x_in_wire.start(40);
}

void readVL6180XStates() 
{
    threads.setSliceMicros(50);
    size_t t = millis();
    while (1)
    {
        {
            Threads::Scope scope(mx);
            vl6180x_in_wire.read();
        }
        while (millis() - t < vl6180x_in_wire.getPeriodMs()) threads.yield();
    }
}

void start() 
{
    threads.addThread(readVL6180XStates, 0, 1024);
}

bool readDists(std::vector<float>& _dists) 
{
    bool result = false;
    {
        Threads::Scope scope(mx);
        if (vl6180x_in_wire.isStable()) 
        {
            result = true;
            _dists = vl6180x_in_wire.getDists();
        }
        else result = false;
    }
    return result;
}

void printDists()
{
    std::vector<float> dists;
    if (!readDists(dists)) 
    {
        Serial.println("Failed to read distances");
        return;
    }
    Serial.print("Dists: ");
    for (size_t i=0; i<dists.size(); ++i)
    {
        Serial.print(dists[i]); Serial.print(" ");
    }
    Serial.println();

}
}