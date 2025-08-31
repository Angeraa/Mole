#pragma once

#include <cmath>
#include <string>

class Wheel {
  public:
    std::string name = "";
    int enc = 0;
    double pos = 0.0;
    double vel = 0.0;
    double cmd = 0.0;
    double rads_per_tick = 0.0;

    Wheel() = default;

    Wheel(const std::string &name, int enc, double rads_per_tick)
      : name(name), enc(enc), rads_per_tick(rads_per_tick) {}

    void initialize(const std::string &name, double ticks_per_rev) {
      this->name = name;
      this->rads_per_tick = (2.0 * M_PI) / ticks_per_rev;
    }

    double calcEncAngle() {
      return enc * rads_per_tick;
    }
};