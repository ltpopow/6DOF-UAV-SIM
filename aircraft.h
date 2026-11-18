#pragma once //multiple inclusion guard

enum StateIndex {
    pn = 0,  //Position NED
    pe,
    pd,
    u,   //Body frame velocity FRD
    v,
    w,
    phi,  // Euler angles
    theta,
    psi, 
    p,   //Body frame angular velocities roll pitch yaw
    q,
    r
};

struct AircraftParameters {
    double mass; //kg
    double gravity; //m/s^2

    double Ixx; //kg*m^2 Inertias
    double Iyy; //kg*m^2
    double Izz; //kg*m^2
    double Ixz; //kg*m^2

    double S; // Wing area (m^2)
    double b; // Wing span (m)
    double c; // Mean chord (m)
    double e; // Oswald efficiency factor

    double CL0; // Lift coefficient @ 0 AoA
    double CLalpha; // Lift coefficient slope
    double CLq; // Lift coefficient due to pitch rate
    double CLde; //Lift coefficient due to elevator deflection

    double CD0; //Drag coefficient @ 0 AoA
    
    double Cm0; //Moment coefficient @ 0 AoA
    double Cmalpha; //Moment coefficient slope
    double Cmq; //Moment coefficient due to pitch rate
    double Cmde; //Moment coefficient due to elevator deflection
    
    double Cy0; //Side force coefficient @ 0 AoA
    double Cybeta; //Side force coefficient slope
    double Cyp; //Side force coefficient due to roll rate
    double Cyr; //Side force coefficient due to yaw rate
    double Cyda; //Side force coefficient due to aileron deflection
    double Cydr; //Side force coefficient due to rudder deflection

    double Cl0; //Roll moment coefficient @ 0 AoA
    double Clbeta; //Roll moment coefficient slope
    double Clp; //Roll moment coefficient due to roll rate
    double Clr; //Roll moment coefficient due to yaw rate
    double Clda; //Roll moment coefficient due to aileron deflection
    double Cldr; //Roll moment coefficient due to rudder deflection

    double Cn0; //Yaw moment coefficient @ 0 AoA
    double Cnbeta; //Yaw moment coefficient slope
    double Cnp; //Yaw moment coefficient due to roll rate
    double Cnr; //Yaw moment coefficient due to yaw rate
    double Cnda; //Yaw moment coefficient due to aileron deflection
    double Cndr; //Yaw moment coefficient due to rudder deflection

};

struct ControlInputs{
    double de; //elevator deflection
    double da; //aileron deflection
    double dr; //rudder deflection
    double T; //thrust
};