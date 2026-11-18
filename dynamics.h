#pragma once //multiple inclusion guard
#include "aircraft.h"

void computeDerivatives(
    const double x[12], 
    double xdot[12], 
    const AircraftParameters& aircraft,
    const ControlInputs& controls);
