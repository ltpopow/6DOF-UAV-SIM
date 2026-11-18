#include "integration.h"
#include "dynamics.h"

void RK4Step(
    double x[12],
    double dt,
    const AircraftParameters& aircraft, 
    const ControlInputs& controls
) //accepting state vector, time step, and aircraft parameters
{
    double k1[12], k2[12], k3[12], k4[12], x_temp[12]; //initialize RK4 steps and temporarty state vector

    computeDerivatives(x, k1, aircraft, controls); //compute k1 with state vector, state derivative, aircraft parameters

    for (int i = 0; i < 12; i++)
    {
        x_temp[i] = x[i] + 0.5 * dt * k1[i];
    }
    computeDerivatives(x_temp, k2, aircraft, controls); //compute k2 

    for (int i = 0; i < 12; i++)
    {
        x_temp[i] = x[i] + 0.5 * dt * k2[i];
    }
    computeDerivatives(x_temp, k3, aircraft, controls); //compute k3 

    for (int i = 0; i < 12; i++)
    {
        x_temp[i] = x[i] + dt * k3[i];
    }
    computeDerivatives(x_temp, k4, aircraft, controls); //compute k4 

    for (int i = 0; i < 12; i++)
    {
        x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]); //combine weighted RK4 estimates to update the state vector
    } 
}
