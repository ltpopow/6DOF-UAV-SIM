#pragma once //multiple inclusion guard
#include "aircraft.h"

// Header function for Range-Kutta 4th order integration method
void RK4Step(
    double x[12],
    double dt,
    const AircraftParameters& aircraft,
    const ControlInputs& controls
);
