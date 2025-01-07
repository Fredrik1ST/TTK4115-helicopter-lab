
%% Preliminary - For Helicopter 1 & 2
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


% Calibration of the encoder and the hardware for the specific helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.40; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.65; % Motor mass [kg]


%% Part 1 - Monovariable Control
% Constants
e_offset = -0.465; % rads from table to horizontal position
p_offset = 0; 
Vs0 = 5.9; % Or around 6.8 - Voltage at which pitch & elevation keeps horizontal equilibrium

K_f_old = ((2*m_p*g) + m_c*g) / Vs0; % Based on sum of forces up vs down
K_f = -g*((m_c*l_c) - (2*m_p*l_h)) / (l_h*Vs0);

L_1 = K_f * l_p;
L_2 = (m_c * l_c - 2*m_p*l_h)*g;
L_3 = K_f * l_h;
L_4 = K_f * l_h;

K_1 = L_1 / (2*m_p*l_p^2);
K_2 = L_3 / (m_c * l_c^2) + (2*m_p*l_h^2);
K_3 = L_4 * Vs0;

% Poles of transfer function
part1_caseNr = 1;
switch part1_caseNr
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

%% Part 2 - Multivariable Control
% 2 Cases:
% 1) Without integral part
% 2) With integral part
part2_caseNr = 2;

switch part2_caseNr
    case 1 % Without integral part
        % Rewrite system in state space
        A = [0 1 0; 0 0 0; 0 0 0];
        B = [0 0; 0 K_1; K_2 0];
        C = [1 0 0; 0 0 0; 0 0 1];
        D = [0 0; 0 0; 0 0];

        % Check controllability (Co = [B; A^1 B; A^2 B; ...; A^n-1 B])
        Co = [B A*B A^2*B]
        rank(Co)

        % Weight matrices for LQR
        % 60 20 10
        Q = [60 0 0; % Weight rate of change in states
             0 20 0; 
             0 0 10]; 

        % 0.5 0.5
        R = [0.5 0; % Weight inputs
             0 0.5];

        % Compute the optimal state feedback LQR controller for the process
        % K: State feedback matrix
        % P: Solution to the corresponding algebraic Riccati equation
        % E: The poles E of the closed-loop system
        [K, S, E] = lqr(A, B, Q, R);
        % F: Calculated from (A-B*K)*x_inf = -B*F*r
        F = [0 K(1,3); K(2,1) 0];

    case 2 % With integral effect
        % Two new states added: gamma (p_c - p) and zeta (e_dot_c - e_dot)
        % Where p_c is the joystick x-axis and e_dot_c is its y-axis
        % New matrix & vector G*r contains p_c & e_dot_c 
        A = [0 1 0 0 0;     % p
             0 0 0 0 0;     % p_dot
             0 0 0 0 0;     % e_dot
            -1 0 0 0 0;     % gamma
             0 0 -1 0 0];   % zeta

        B = [0   0; 
             0   K_1;
             K_2 0; 
             0   0; 
             0   0];

        C = [1 0 0 0 0;
             0 0 0 0 0; 
             0 0 1 0 0;
             0 0 0 0 0;
             0 0 0 0 0];

        D = [0 0; 0 0; 0 0; 0 0; 0 0];
        G = [0 0; 0 0; 0 0; 0 0; 0 1;];

        % Check controllability (Co = [B; A^1 B; A^2 B; ...; A^n-1 B])
        %disp("[B AB A²B]");
        Co = [B A*B A^2*B]; % Controllable since Co is full rank
        CoRank = rank(Co);

        % New weight matrices for LQR + integral effect
        % 90 40 90 50 70
        % 45 20 45 20 35
        Q = [9 0 0 0 0;
             0 4 0 0 0; 
             0 0 9 0 0;
             0 0 0 5 0;
             0 0 0 0 7]; % Weight rate of change in states
        % 0.5 0.5
        
        R = [0.5 0; 
             0 0.5]; % Weight inputs
         

%        Q = [3 0 0 0 0;
%             0 1 0 0 0; 
%             0 0 5 0 0;
%             0 0 0 5 0;
%             0 0 0 0 5];
%         
%         R = [0.1 0; 
%              0 0.1];
        [K, S, E] = lqr(A, B, Q, R);

        %Find poles
        A_c = A-B*K;
        sys_c = ss(A_c, B, C, 0)
        [p_lqr,z_lqr] = pzmap(sys_c);
        p_lqr
        
        %F = [0 K(1,3); K(2,1) 0];
        F = [0 1; 1 0]; % Try without feedforward gains
end


%% Part 3 - Luenberger State Observer

% constants
IMU_pitch_offset = 0; %0.0216;
IMU_elevation_offset = 0; %0.5026;

PORT = 3;

A_sys = [
    0 1 0 0 0;      % p
    0 0 0 0 0;      % p_dot
    0 0 0 1 0;      % e
    0 0 0 0 0;      % e_dot
    K_3 0 0 0 0];   % lambda_dot

B = [0    0;        % V~_s = K_2*e_dot   
     0    K_1;      % V_d = K_1*p_dot
     0    0;
     K_2  0;        
     0    0];

 C_est = [
     1 0 0 0 0;     % p
     0 0 0 0 0;     % p_dot
     0 0 1 0 0;     % e
     0 0 0 0 0;     % e_dot
     0 0 0 0 1];    % lambda_dot

A_est = [
    0 1 0 0 0;      % p
    0 0 0 0 0;      % p_dot
    0 0 0 1 0;      % e
    0 0 0 0 0;      % e_dot
    K_3 0 0 0 0];   % lambda_dot




% Create poles for estimator
% Rules of thumb:
% Largest pole should be 2-20x times the largest pole of the system
% Spread them evenly as a half circle on the negative half of the plane

% p_multiplier = 5;
% p_radius = min(p_lqr) * p_multiplier;
% phi = pi/4; % Spread around +/- phi radians
% spread = -phi:(phi/(2)):phi;
% p_est = p_radius*exp(1i*spread);
% 
% plot(real(p_lqr),imag(p_lqr),'or',...
%     real(p_est),imag(p_est),'kx');grid on;axis equal
% legend("System poles", "Observer poles")
% 
% % p1:
% % lqr: [-2.09 -1.65 -0.91 -4.38 -0.90]
% % obs: largest pole * 5, phi = +/- pi/4 rads - see plot above
% 
% L = place(A_est', C_est', p_est).';

% Check observability
m_obs = obsv(A_sys,C_est);
rank_m_obs = rank(m_obs)


%% Part 3 subtask - Calculate offset

% IMU = 1;
% enc = 3;
% offset = mean(logDat(:,enc))-mean(IMU_angles(:,IMU))
% 
% IMU_with_offset = IMU_angles(:,IMU)+offset;
% 
% figure
% 
% %Original
% subplot(2,1,1)
% title("IMU vs encoder (Pitch angle)")
% xlabel("Time [ms]")
% ylabel("Angle [rad]")
% hold on
% plot(IMU_angles(:,IMU))
% plot(logDat(:,enc))
% 
% % with offset
% subplot(2,1,2)
% title("IMU with offset vs encoder (Pitch angle)")
% xlabel("Time [ms]")
% ylabel("Angle [rad]")
% hold on
% plot(IMU_with_offset)
% plot(logDat(:,enc))
%% Plot states

% ts = 1000;
% t = 1:1:length(logDat(:,1));
% 
% figure(1)
% hold on
% title("IMU Data vs Encoder Data")
% plot(t,logDat(:,1))
% plot(t,logDat(:,2))
% plot(t,logDat(:,3))
% plot(t,logDat(:,4))
% plot(t,logDat(:,5))
% plot(t,logDat(:,6))
% 
% plot(t, IMU_angles(:,1))
% plot(t, IMU_angles(:,2))
% 
% plot(t, eulerRates(:,1))
% plot(t, eulerRates(:,2))
% plot(t, eulerRates(:,3))
% 
% legend("Travel [rad]","Travel rate [rad/s]","Pitch [rad]",...
% "Pitch rate [rad/s]","Elevation [rad]","Elevation rate [rad/s]",...
% "IMU Pitch [rad]", "IMU Elevation [rad]",...
% "IMU Pitch rate [rad/s]", "IMU Elevation rate [rad/s]", "IMU Travel rate [rad/s]")
% 

%% Plot encoder vs estimator

ts = 1000;
t = 1:1:length(encDat(:,1));

figure(1)
hold on
title("Observer vs Encoder Data")
xlabel('Time')
ylabel('Angle (rad)')
plot(t, encDat(:,1)) % Encoder pitch
plot(t, encDat(:,2)) % Encoder pitch rate
plot(t, encDat(:,3)) % Encoder elevation
plot(t, encDat(:,4)) % Encoder elevation rate
plot(t, encDat(:,5)) % Encoder travel
plot(t, encDat(:,6)) % Encoder travel rate

plot(t, IMU_angles(:,1))        % IMU pitch
plot(t, IMU_eulerRates(:,1))    % IMU pitch rate
plot(t, IMU_angles(:,2))        % IMU elevation
plot(t, IMU_eulerRates(:,2))    % IMU elevation rate
plot(t, IMU_eulerRates(:,3))    % IMU travel rate

plot(t, obsDat(:,1))    % Observer pitch
plot(t, obsDat(:,2))    % Observer pitch rate
plot(t, obsDat(:,3))    % Observer elevation
plot(t, obsDat(:,4))    % Observer elevation rate
plot(t, obsDat(:,5))    % Observer travel rate

legend("Travel [rad]","Travel rate [rad/s]","Pitch [rad]", "Pitch rate [rad/s]","Elevation [rad]","Elevation rate [rad/s]",...
"IMU Pitch [rad]", "IMU Pitch rate [rad/s]","IMU Elevation [rad]", "IMU Elevation rate [rad/s]", "IMU Travel rate [rad/s]",...
"Obs Pitch [rad]", "Obs Pitch rate [rad/s]", "Obs Elevation [rad]", "Obs Elevation rate [rad/s]", "Obs Travel rate [rad/s]")

%% Part 4 - Kalman Filter
if runPart(4)

    A_k = [
        0 1 0 0 0;      % p
        0 0 0 0 0;      % p_dot
        0 0 0 1 0;      % e
        0 0 0 0 0;      % e_dot
        0 0 0 0 1;      % lambda
        K_3 0 0 0 0];   % lambda_dot
    
    B_k = [
        0    0;         % V~_s = K_2*e_dot
        0    K_1;       % V_d = K_1*p_dot
        0    0;
        K_2  0;
        0    0;
        0    0];

    C_k = [
        1 0 0 0 0 0;    % p
        0 1 0 0 0 0;    % p_dot
        0 0 1 0 0 0;    % e
        0 0 0 1 0 0;    % e_dot
        0 0 0 0 1 0;    % lambda
        0 0 0 0 0 1];   % lambda_dot
end