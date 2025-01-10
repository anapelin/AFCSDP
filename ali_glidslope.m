clear;

global fi_flag_Simulink

newline = sprintf('\n');

%% This is necessary so as to trim the aircraft
x_a = 6.1 * 0.3048; %in meters

%% Slant Range
% This range is adjusted so that glide slope is intercepted after 10
% seconds of simulation, x_i = 3000 ft, h_i = 2000 ft
R = sqrt(5550 ^ 2 + 2000 ^ 2);

%% Trim aircraft to desired altitude and velocity
%%
altitude = 5000;
velocity = 300;

FC_flag = 1; % Trim for steady wings-level flight

%% Initial guess for trimcx

thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity

%disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find trim for lofi model at desired altitude and velocity

disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.

trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);

SS_lo = linearize('LIN_F16Block');

%% Selecting the necessary states
% For glide slope, these are: altitude, true airspeed, angle of attack,
% pitch attitude angle, pitch rate, elevator state. thrust state
% With indices [3 7 8 5 11 13 14]

glide_states = [3 7 8 5 11 13 14];
glide_in = [1 2];
glide_out = [3 7 8 5 11];

glide_trim_state = trim_state_lo(glide_out, :);

trim_state_glide = nan(7,1);
trim_state_glide([6, 7]) = [trim_thrust_lo, trim_control_lo(1)];
trim_state_glide(isnan(trim_state_glide)) = glide_trim_state;

clear glide_trim_state;

%% Pole Analysis for the system
% Alpha Feedback
glide_A = SS_lo.A(glide_states, glide_states);
glide_B = SS_lo.B(glide_states, 2);
glide_C = SS_lo.C(glide_out, glide_states);

[glide_num, glide_den]  = ss2tf(glide_A, glide_B, glide_C(3, :), 0); % alpha-elevator response

delta_e_to_alpha = tf(glide_num, glide_den);

figure
pzmap(delta_e_to_alpha);
title(delta_e_to_alpha);


alpha_noise = tf(1, [0.1 1]);

k_alpha = -0.64649; % This is selected from C when H is alpha_noise

%sisotool(delta_e_to_alpha)

%closing the alpha loop
glide_A_cl = glide_A - glide_B*k_alpha*glide_C(3,:);

% SISO for q feedback
qfb = ss(glide_A_cl,glide_B,glide_C(5,:),0);

[delta_e_to_q_num, delta_e_to_q_den]  = ss2tf(qfb.A, qfb.B, qfb.C, qfb.D);

delta_e_to_q = tf(delta_e_to_q_num, delta_e_to_q_den);

%sisotool(delta_e_to_q)

k_q = -0.0079109;

% Closing the alpha-q-loop

glide_A_sas = glide_A_cl - glide_B*k_q*glide_C(5,:);

%% Pole Analysis
 glide_A = glide_A_sas;
 glide_B = SS_lo.B(glide_states, glide_in);
 glide_C = SS_lo.C(glide_out, glide_states);
 glide_D = SS_lo.D(glide_out, glide_in);

%% Constructing the state-space system
glide_sys = ss(glide_A, ...
    glide_B, ...
    glide_C, ...
    glide_D, 'StateName', ...
    SS_lo.StateName(glide_states), 'InputName', ...
    SS_lo.InputName(glide_in), 'Name', 'F16 Glide Slope Hold Mode');

%% Getting the states for flight path transfer
tfs = tf(glide_sys);
delta_e_q = tfs(5, 2);
delta_e_theta = tfs(4, 2);
delta_e_alpha = tfs(3, 2);

s = tf('s');

theta_gamma = minreal(1 - minreal(delta_e_theta) / minreal(delta_e_alpha));
theta_gamma

k_q_cas = -0.10753; %determined from the sisotool

q_closed = k_q_cas * delta_e_q / (1 + k_q_cas * delta_e_q);

theta_open = 1 / s * q_closed;

k_theta_cas = -2; % determined using the sisotool


disp(eig(glide_sys.A));

