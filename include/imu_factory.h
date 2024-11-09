#pragma once

#include <tuple>
#include <IMUBase.hpp>

IMUBase* GetImu(TwoWire& wire);
std::tuple<float, float, float> QuatToEuler(float q0, float q1, float q2, float q3);
