% FOR HELICOPTER NR 1-2
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.40; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.65; % Motor mass [kg]


%%%%%%%%%%% Problem 01 - Monovariable Control
e_offset = -0.465; % rads from table to horizontal position
p_offset = -0; 
Vs0 = 5.9; % Or around 6.8 - Voltage at which pitch & elevation keeps horizontal equilibrium

K_f = ((2*m_p*g) + m_c*g) / Vs0; % Based on sum of forces up vs down

L_1 = K_f * l_p;
L_2 = (m_c * l_c - 2*m_p*l_h)*g;
L_3 = K_f * l_h;
L_4 = K_f * l_h;

K_1 = L_1 / (2*m_p*l_p^2);
K_2 = L_3 / (m_c * l_c^2) + (2*m_p*l_h^2);
K_3 = L_4 * Vs0;

% Poles of transfer function
caseNr = 6;
switch caseNr
    case 1 % Asymptotically stable, without oscillation (critically damped)
        pole_1 = -10;
        pole_2 = -10;
    case 2 % Stable without oscillation (overdamped)
        pole_1 = -5;
        pole_2 = -10;
    case 3 % Stable with oscillations (underdamped)
        pole_1 = -6 + 3i;
        pole_2 = -6 - 3i;
    case 4 % Marginally stable
        pole_1 = 0;
        pole_2 = -5;
    case 5 % Very unstable without oscillation
        pole_1 = 5;
        pole_2 = 5;
    case 6 % Unstable with oscillations
        pole_1 = 2 + 3i;
        pole_2 = 2 - 3i;
end

K_pp = pole_1 * pole_2 / K_1;
K_pd = -(pole_1 + pole_2) / K_1;