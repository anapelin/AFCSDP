%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
% Edit: Ewoud Smeur (2021)
%================================================
clear;

global fi_flag_Simulink

newline = sprintf('\n');

rng(1,"twister");

%% Trim aircraft to desired altitude and velocity
%%
altitude = 15000;
velocity = 500;

FC_flag = 1; % Trim for steady wings-level flight

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for lofi model at desired altitude and velocity

disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block');

newline = sprintf('\n');

%% System state names
state_names = {
    'npos', 
    'epos', 
    'h', 
    'phi', 
    'theta', 
    'psi', 
    'Vt', 
    'alpha', 
    'beta', 
    'p', 
    'q', 
    'r', 
    'thrust_state', 
    'elevator_state', 
    'aileron_state', 
    'rudder_state' 
};
%% Decoupled states
%Vt,α,θ,q, de, dt -> 7, 8, 5, 11, 14, 13
%β,ϕ,p,r,da, dr -> 9, 4, 10, 12, 15, 16

long_indices_states = [7 8 5 11 14 13];
long_indices_states_wo = [1 2 3 4];
long_indices_input = [2 1];
long_indices_output = [8 11];
lat_indices_states = [9 4 10 12 15 16];
lat_indices_states_wo = [1 2 3 4];
lat_indices_input = [3 4];
lat_indices_output = [9 12];


%% Declare space state matrices
A_full = SS_lo.A;
B_full = SS_lo.B;
C_full = SS_lo.C;
D_full = SS_lo.D;

% Extract longitudinal dynamics
A_long = A_full(long_indices_states, long_indices_states);
B_long = B_full(long_indices_states, long_indices_input);
C_long = C_full(long_indices_output, long_indices_states);
D_long = D_full(long_indices_output, long_indices_input);

% Extract lateral dynamics
A_lat = A_full(lat_indices_states, lat_indices_states);
B_lat = B_full(lat_indices_states, lat_indices_input);
C_lat = C_full(lat_indices_output, lat_indices_states);
D_lat = D_full(lat_indices_output, lat_indices_input);

% indices of the new matrices
indices_states = [1 2 3 4];
indices_outputs = [5 6];

A_ac_long = A_long(long_indices_states_wo, long_indices_states_wo);
B_ac_long = A_long(long_indices_states_wo, indices_outputs);
A_ac_lat = A_lat(lat_indices_states_wo, lat_indices_states_wo);
B_ac_lat = A_lat(lat_indices_states_wo, indices_outputs);

%% Finding the eigenvalues 
% Longitudinal eigenvalues
eigs_long = eig(A_ac_long);
sp_eig = eigs_long([1 2]);
ph_eig = eigs_long([3 4]);

% Lateral eigenvalues;
eigs_lat = eig(A_ac_lat);
dr_eig = eigs_lat([1 2]);
ar_eig = eigs_lat(3);
spiral_eig = eigs_lat(4);


%% Calculate Parameters for each eigen motion
% Short period
sp_nf = abs(sp_eig(1));
sp_damp = - real(sp_eig(1))/sp_nf;
sp_p = 2*pi/abs(imag(sp_eig(1)));
sp_t12 = log(0.5)/real(sp_eig(1));

% Phugoid
ph_nf = abs(ph_eig(1));
ph_damp = - real(ph_eig(1))/ph_nf;
ph_p = 2*pi/abs(imag(ph_eig(1)));
ph_t12 = log(0.5)/real(ph_eig(1));

% Dutch Roll
dr_nf = abs(dr_eig(1));
dr_damp = - real(dr_eig(1))/dr_nf;
dr_p = 2*pi/abs(imag(dr_eig(1)));
dr_t12 = log(0.5)/real(dr_eig(1));

% Aperiodic Roll
ar_nf = abs(ar_eig(1));
ar_t12 = log(0.5)/real(ar_eig(1));
ar_tau = -1 /real(ar_eig(1));

% Spirnf = abs(spiral_eig(1));
spiral_t12 = log(0.5)/real(spiral_eig(1));
spiral_tau = -1 /real(spiral_eig(1));

% Create state-space models
sys_long = ss(A_long, B_long, C_long, D_long);
sys_lat = ss(A_lat, B_lat, C_lat,D_lat);

% Display the reduced systems

% TODO: replace indexes of [V, alpha, theta, q] and [beta, phi, p, r]
% TODO: Plot the different eigenmotion time responses to support the numbers calculated above

% Short period response 
figure





