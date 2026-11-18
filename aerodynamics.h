#pragma once //multiple inclusion guard
#include "aircraft.h"

double calculateAirspeed(const double x[12]); //input state vector
double calculateAlpha(const double x[12]); 
double calculateBeta(const double x[12]); 
double calculateDensity(const double x[12]);
double calculateDynamicPressure(const double x[12]);

//Aerodynamic Coefficients
double calculateCL(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls); //state vector, aircraft parameters, elevator control
double calculateCD(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);
double calculateCm(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);
double calculateCy(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);
double calculateCl(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);
double calculateCn(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);


//World Frame Forces
double calculateLift(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls); 
double calculateDrag(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls);