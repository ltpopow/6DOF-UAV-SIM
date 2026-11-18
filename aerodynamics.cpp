#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "Resources/international-standard-atmosphere-master/ISA.hpp"
#include "aerodynamics.h"
#include "aircraft.h"
using namespace std;

double calculateAirspeed(const double x[12]) //input state vector
{
    return sqrt(x[u] * x[u] + x[v] * x[v] + x[w] * x[w]); //body frameairspeed magnitude
} 

double calculateAlpha(const double x[12]) 
{
    return atan2(x[w], x[u]); //body frame AoA
}

double calculateBeta(const double x[12]) 
{
    double V = calculateAirspeed(x); 
    if(V < 1e-4)
    {
        return 0.0; //avoid division by zero
    }
    else
    {
        return asin((clamp(x[v] / V, -1.0, 1.0))); //body frame sideslip angle, clamp for rounding errors
    }
}

double calculateDensity(const double x[12]) 
{
    double altitude = -x[pd]; //NED position to altitude

    auto atmosphere = LouiEriksson::ISA<double>::TrySolve(altitude); //atmospheric properties @ altitude

    if (!atmosphere.has_value()) 
    {
        throw std::runtime_error("ISA Calc Failed"); //throw an error if unable to calculate
    }

    return atmosphere->m_Density; //density in kg/m^3
}

double calculateDynamicPressure(const double x[12])
{
    double rho = calculateDensity(x); //density
    double V = calculateAirspeed(x); //airspeed

    return 0.5 * rho * V * V; //dynamic pressure (Pa)
}

double calculateQhat(const double x[12], const AircraftParameters& aircraft) 
{
    double V = calculateAirspeed(x); //airspeed

    if (V < 1e-4) 
    {
        return 0.0; //avoid division by zero
    }
    else 
    {
        return x[q] * aircraft.c / (2.0 * V); //nondimensional pitch rate
    }
}

double calculatePhat(const double x[12], const AircraftParameters& aircraft) 
{
    double V = calculateAirspeed(x); //airspeed

    if (V < 1e-4) 
    {
        return 0.0; //avoid division by zero
    }
    else 
    {
        return x[p] * aircraft.b / (2.0 * V); //nondimensional roll rate
    }
}

double calculateRhat(const double x[12], const AircraftParameters& aircraft) 
{
    double V = calculateAirspeed(x); //airspeed

    if (V < 1e-4) 
    {
        return 0.0; //avoid division by zero
    }
    else 
    {
        return x[r] * aircraft.b / (2.0 * V); //nondimensional yaw rate
    }
}

//Aerodynamic Coefficients
//Longitudinal
double calculateCL(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls)
{
    double alpha = calculateAlpha(x); //AoA
    double V = calculateAirspeed(x); //airspeed
    double qhat = calculateQhat(x, aircraft); //nondimensional pitch rate

    return aircraft.CL0 + aircraft.CLalpha * alpha + aircraft.CLq * qhat + aircraft.CLde * controls.de; //lift coefficient
}

double calculateCD(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) 
{
    constexpr double pi = 3.14159265358979323846; //pi constant
    
    double CL = calculateCL(x, aircraft, controls); //lift coefficient
    double AR = aircraft.b * aircraft.b / aircraft.S; //aspect ratio
    double k = 1.0 / (pi * aircraft.e * AR); //induced drag factor

    return aircraft.CD0 + k * (CL * CL); //drag coefficient
}

double calculateCm(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls)
{
    double alpha = calculateAlpha(x); //AoA
    double V = calculateAirspeed(x); //airspeed
    double qhat = calculateQhat(x, aircraft); //nondimensional pitch rate

    return aircraft.Cm0 + aircraft.Cmalpha * alpha + aircraft.Cmq * qhat + aircraft.Cmde * controls.de; //moment coefficient
}

//Lateral
double calculateCy(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) 
{
    double beta = calculateBeta(x); //sideslip angle
    double phat = calculatePhat(x, aircraft); //nondimensional roll rate
    double rhat = calculateRhat(x, aircraft); //nondimensional yaw rate

    return aircraft.Cy0 + aircraft.Cybeta * beta + aircraft.Cyp * phat + aircraft.Cyr * rhat + aircraft.Cyda * controls.da + aircraft.Cydr * controls.dr; //side force coefficient
}

double calculateCl(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) 
{
    double beta = calculateBeta(x); //sideslip angle
    double phat = calculatePhat(x, aircraft); //nondimensional roll rate
    double rhat = calculateRhat(x, aircraft); //nondimensional yaw rate

    return aircraft.Cl0 + aircraft.Clbeta * beta + aircraft.Clp * phat + aircraft.Clr * rhat + aircraft.Clda * controls.da + aircraft.Cldr * controls.dr; //roll moment coefficient
}

double calculateCn(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) 
{
    double beta = calculateBeta(x); //sideslip angle
    double phat = calculatePhat(x, aircraft); //nondimensional roll rate
    double rhat = calculateRhat(x, aircraft); //nondimensional yaw rate

    return aircraft.Cn0 + aircraft.Cnbeta * beta + aircraft.Cnp * phat + aircraft.Cnr * rhat + aircraft.Cnda * controls.da + aircraft.Cndr * controls.dr; //yaw moment coefficient
}

//World Frame Forces
double calculateLift(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) //world frame
{
    double qinf = calculateDynamicPressure(x); //dynamic pressure
    return qinf * aircraft.S * calculateCL(x, aircraft, controls); //lift force (N)
}

double calculateDrag(const double x[12], const AircraftParameters& aircraft, const ControlInputs& controls) //world frame
{
    double qinf = calculateDynamicPressure(x); //dynamic pressure
    return qinf * aircraft.S * calculateCD(x, aircraft, controls); //drag force (N)
}

//World Frame Moments
