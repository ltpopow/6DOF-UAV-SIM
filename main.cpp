#include <iostream>
#include "aircraft.h"
#include "dynamics.h"
#include "integration.h"
#include "aerodynamics.h"
using namespace std;



int main (void)
{
  constexpr double pi = 3.14159265358979323846; //pi constant
  
  //Aircraft parameters
  AircraftParameters aircraft;
  aircraft.mass = 13.5; // kg
  aircraft.gravity = 9.81; // m/s^2

  aircraft.Ixx = 0.8244; // kg m^2
  aircraft.Iyy = 1.135; // kg m^2
  aircraft.Izz = 1.759; // kg m^2
  aircraft.Ixz = 0.1204; // kg m^2    

  aircraft.S = 0.55; // Wing area (m^2)
  aircraft.b = 2.8956; // Wing span (m)
  aircraft.c = 0.18994; // Mean chord (m)
  aircraft.e = 0.9; // Oswald efficiency factor

  aircraft.CL0 = 0.28; // Lift coefficient 
  aircraft.CLalpha = 3.45; // Lift coefficient slope
  aircraft.CLq = 0.0; // Lift coefficient due to pitch rate
  aircraft.CLde = -0.36; // Lift coefficient due to elevator deflection

  aircraft.CD0 = 0.03; // Drag coefficient 

  aircraft.Cm0 = -0.02338; // Moment coefficient 
  aircraft.Cmalpha = -0.38; // Moment coefficient slope
  aircraft.Cmq = -3.6; // Moment coefficient due to pitch rate
  aircraft.Cmde = -0.5; // Moment coefficient due to elevator deflection
  
  aircraft.Cy0 = 0.0; // Side force coefficient 
  aircraft.Cybeta = -0.98; // Side force coefficient slope
  aircraft.Cyp = 0.0; // Side force coefficient due to roll rate
  aircraft.Cyr = 0.0; // Side force coefficient due to yaw rate
  aircraft.Cyda = 0.0; // Side force coefficient due to aileron deflection
  aircraft.Cydr = -0.17; // Side force coefficient due to rudder

  aircraft.Cl0 = 0.0; // Roll moment coefficient 
  aircraft.Clbeta = -0.12; // Roll moment coefficient slope
  aircraft.Clp = -0.26; // Roll moment coefficient due to roll rate
  aircraft.Clr = 0.14; // Roll moment coefficient due to yaw rate
  aircraft.Clda = 0.08; // Roll moment coefficient due to aileron deflection
  aircraft.Cldr = 0.105; // Roll moment coefficient due to rudder def

  aircraft.Cn0 = 0.0; // Yaw moment coefficient 
  aircraft.Cnbeta = 0.25; // Yaw moment coefficient slope
  aircraft.Cnp = 0.022; // Yaw moment coefficient due to roll rate
  aircraft.Cnr = -0.35; // Yaw moment coefficient due to yaw rate
  aircraft.Cnda = 0.06; // Yaw moment coefficient due to aileron deflection
  aircraft.Cndr = -0.032; // Yaw moment coefficient due to rudder

  //Aircraft state vector and state derivative vector
  double x[12] = {0};     
  double xdot[12] = {0};  

  //initial conditions
  x[pn] = 0.0; //position world frame (NED)
  x[pe] = 0.0;
  x[pd] = -100.0;
  
  x[u] = 24.898042;  //velocity body frame (FRD)
  x[v] = 0.0;
  x[w] = 2.255551;

  x[phi] = 0.0 * pi / 180.0;  //roll, pitch, yaw between body and world frame
  x[theta] = 5.148 * pi / 180.0;
  x[psi] = 0.0 * pi / 180.0;

  x[p] = 0.0; //roll, pitch, yaw rates, body frame 
  x[q] = 0.0; 
  x[r] = 0.0;

  //Controls
  ControlInputs controls;
  controls.de = -6.592 * pi / 180.0;
  controls.da = 0.0 * pi / 180.0;
  controls.dr = 0.0 * pi / 180.0;
  controls.T = 8.21955; 
  
  //Simulation settings
  double t0 = 0.0; //seconds
  double total_time = 20.0; 
  double dt = .001;   
  double steps = total_time / dt; //number of steps

computeDerivatives(x, xdot, aircraft, controls);

std::cout << "u_dot  = " << xdot[u] << '\n';
std::cout << "w_dot  = " << xdot[w] << '\n';
std::cout << "q_dot  = " << xdot[q] << '\n';
std::cout << "pd_dot = " << xdot[pd] << '\n';
  int i = 0; 
  //numerical integration using RK4 to calcualate state derivatives and update state vector
  while (t0 < total_time)
  {
    RK4Step(x, dt, aircraft, controls); 
    t0 += dt;
    i++;
    if ((i + 1) % 1000 == 0)
    {
      cout << "t = " << (i + 1) * dt
          << " | Va = " << calculateAirspeed(x)
          << " | beta = " << calculateBeta(x) * 180.0 / 3.141592653589793
          << " | phi = " << x[phi] * 180.0 / 3.141592653589793
          << " | psi = " << x[psi] * 180.0 / 3.141592653589793
          << " | p = " << x[p]
          << " | r = " << x[r]
          << " | Y = " << calculateDynamicPressure(x) * aircraft.S
                    * calculateCy(x, aircraft, controls)
          << " | L = " << calculateDynamicPressure(x) * aircraft.S * aircraft.b
                    * calculateCl(x, aircraft, controls)
          << " | N = " << calculateDynamicPressure(x) * aircraft.S * aircraft.b
                    * calculateCn(x, aircraft, controls)
                    << " | altitude = " << -x[pd]
          << " | theta = " << x[theta] * 180.0 / pi
          << " | alpha = " << calculateAlpha(x) * 180.0 / pi
          << " | q = " << x[q]
          << " | Lift = " << calculateLift(x, aircraft, controls)
          << " | Drag = " << calculateDrag(x, aircraft, controls)
          << " | M = " << calculateDynamicPressure(x) * aircraft.S * aircraft.c
            * calculateCm(x, aircraft, controls)
          << endl;

    }
  }
  
  return 0;
}