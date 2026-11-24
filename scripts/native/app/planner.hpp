#pragma once
#include "model.hpp"
namespace pc {
struct Plan {
    M inputs;
    Vec slacks;
    double tracking = inf, stability = inf;
};
struct Decision {
    Plan plan;
    std::string source;
    int attempts = 0, window = 0, avoidance = 0;
    double solveSeconds = 0;
};
int warmup(const Config &c);
Decision plan(const std::vector<Member> &members, bool active, bool previousActive, const Plan &prior,
              const U &previous, const Curve &curve, const U &trimOffset, const Config &cfg,
              Clock::time_point start);
} // namespace pc
