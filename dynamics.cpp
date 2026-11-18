#include <cmath>
#include "dynamics.h"
#include "aerodynamics.h"

void computeDerivatives(
    const double x[12], //state vector
    double xdot[12], //state derivative vector
    const AircraftParameters& aircraft, //aircraft parameters
    const ControlInputs& controls //control inputs
)
{    
  //Determinant of inertia matrix
  double inertia_det = aircraft.Ixx * aircraft.Izz - aircraft.Ixz * aircraft.Ixz; 

  
  //Gravity, body frame
  double Fxg = 0.0 - (aircraft.mass * aircraft.gravity) * sin(x[theta]);
  double Fyg = 0.0 + (aircraft.mass * aircraft.gravity) * cos(x[theta]) * sin(x[phi]);
  double Fzg = 0.0 + (aircraft.mass * aircraft.gravity) * cos(x[theta]) * cos(x[phi]);

  //Aerodynamic Forces, body frame
  double qinf = calculateDynamicPressure(x); //dynamic pressure

  double Drag = calculateDrag(x, aircraft, controls); //drag force
  double Lift = calculateLift(x, aircraft, controls); //lift force

  double alpha = calculateAlpha(x); //angle of attack

  double Fxa = - Drag * cos(alpha) + Lift * sin(alpha); 
  double Fya = qinf * aircraft.S * calculateCy(x, aircraft, controls); 
  double Fza = - Drag * sin(alpha) - Lift * cos(alpha);
  
  //Forces, body frame
  double X = Fxa + Fxg + controls.T; 
  double Y = Fya + Fyg;
  double Z = Fza + Fzg;

  //Moments
  double Cm = calculateCm(x, aircraft, controls); //Pitch
  double M = Cm * qinf * aircraft.S * aircraft.c; 
  double Cl = calculateCl(x, aircraft, controls); //Roll
  double L = Cl * qinf * aircraft.S * aircraft.b; 
  double Cn = calculateCn(x, aircraft, controls); //Yaw
  double N = Cn * qinf * aircraft.S * aircraft.b; 

  //Translational accelerations
  double udot = X / aircraft.mass + x[r] * x[v] - x[q] * x[w];
  double vdot = Y / aircraft.mass + x[p] * x[w] - x[r] * x[u];
  double wdot = Z / aircraft.mass + x[q] * x[u] - x[p] * x[v];

  //Rotational accerlations
  double pdot = (aircraft.Izz * L + aircraft.Ixz * N) / inertia_det + x[q] * x[p] * (aircraft.Ixz * (aircraft.Ixx - aircraft.Iyy + aircraft.Izz)) / inertia_det +  x[r] * x[q] * (aircraft.Izz * (aircraft.Iyy - aircraft.Izz) - (aircraft.Ixz * aircraft.Ixz)) / inertia_det; 
  double qdot = (M + (aircraft.Izz - aircraft.Ixx) * x[p] * x[r] + (aircraft.Ixz * (x[r] * x[r] - x[p] * x[p]))) / aircraft.Iyy;
  double rdot = (aircraft.Ixz * L + aircraft.Ixx * N) / inertia_det + x[p] * x[q] * (aircraft.Ixx * (aircraft.Ixx - aircraft.Iyy) + aircraft.Ixz * aircraft.Ixz) / inertia_det - x[r] * x[q] * (aircraft.Ixz * (aircraft.Ixx - aircraft.Iyy + aircraft.Izz)) / inertia_det;

  //Euler angle rates
  double phidot = x[p] + (x[q] * sin(x[phi]) + x[r] * cos(x[phi])) * tan(x[theta]);
  double thetadot = x[q] * cos(x[phi]) - x[r] * sin(x[phi]);
  double psidot = (x[q] * sin(x[phi]) + x[r] * cos(x[phi])) / cos(x[theta]);

  //NED position rates
  double pndot = x[u] * cos(x[theta]) * cos(x[psi]) + x[v] * (sin(x[phi]) * sin(x[theta]) * cos(x[psi]) - cos(x[phi]) * sin(x[psi])) + x[w] * (cos(x[phi]) * sin(x[theta]) * cos(x[psi]) + sin(x[phi]) * sin(x[psi]));
  double pedot = x[u] * cos(x[theta]) * sin(x[psi]) + x[v] * (sin(x[phi]) * sin(x[theta]) * sin(x[psi]) + cos(x[phi]) * cos(x[psi])) + x[w] * (cos(x[phi]) * sin(x[theta]) * sin(x[psi]) - sin(x[phi]) * cos(x[psi]));
  double pddot = -x[u] * sin(x[theta]) + x[v] * sin(x[phi]) * cos(x[theta]) + x[w] * cos(x[phi]) * cos(x[theta]);
  
  xdot[pn] = pndot;
  xdot[pe] = pedot;
  xdot[pd] = pddot;
  xdot[u] = udot;
  xdot[v] = vdot;
  xdot[w] = wdot;
  xdot[phi] = phidot;
  xdot[theta] = thetadot;
  xdot[psi] = psidot;
  xdot[p] = pdot;
  xdot[q] = qdot;
  xdot[r] = rdot;
}