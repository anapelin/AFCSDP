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

global fi_flag_Simulinkn

newline = sprintf('\n');
close all

rng(1,"twister");

%% Trim aircraft to desired altitude and velocity
%%
altitude = 40000;
velocity = 300;

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
%β,ϕ,p,r,da,dr -> 9, 4, 10, 12, 15, 16

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudinal Eigenmotions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_ac_long = C_long([1 2], [1 2 3 4]);
D_ac_long = zeros(2, 2);

% Full longitudinal state-space model (4 states: Vt, alpha, theta, q)
sys_long_full = ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);

% Get the eigenvector corresponding to the short-period mode
[long_eigvecs, ~] = eig(A_ac_long);
%[num_states, ~] = size(sys_long_full.A);
%disp(['Number of states: ', num2str(num_states)]);

%%%%%%%%%%%%%%%
%% Short Period
%%%%%%%%%%%%%%%

% Choose the eigenvector corresponding to the short-period mode
sp_init_cond = real(long_eigvecs(:, 1)); % Short-period eigenvector (real part)
%sp_init_cond = [0,0,0,0];

% Create the time vector for simulation
t = 0:0.01:10;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for elevator and throttle 
u(:, 1) = -1; % Negative elevator step input

% Simulate the response of the system using lsim
[y_sp, t_out, x_sp] = lsim(sys_long_full, u, t, sp_init_cond);

% Plot V_t (Speed)
figure;
plot(t_out, x_sp(:, 1), 'b', 'LineWidth', 1.5);
title('Short Period Time Response: V_t (Speed)');
xlabel('Time (s)');
ylabel('V_t');
grid on;

% Plot \alpha (Angle of Attack)
figure;
plot(t_out, x_sp(:, 2), 'r', 'LineWidth', 1.5);
title('Short Period Time Response: \alpha (Angle of Attack)');
xlabel('Time (s)');
ylabel('\alpha (deg)');
grid on;

% Plot \theta (Pitch Angle)
figure;
plot(t_out, x_sp(:, 3), 'g', 'LineWidth', 1.5);
title('Short Period Time Response: \theta (Pitch Angle)');
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;

% Plot q (Pitch Rate)
figure;
plot(t_out, x_sp(:, 4), 'm', 'LineWidth', 1.5);
title('Short Period Time Response: q (Pitch Rate)');
xlabel('Time (s)');
ylabel('q (deg/s)');
grid on;

%%%%%%%%%%
%% Phugoid
%%%%%%%%%%

ph_init_cond = real(long_eigvecs(:, 3));
%ph_init_cond = [0,0,0,0];


% Create the time vector for simulation
t = 0:0.01:150;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for elevator and throttle 
u(:, 1) = -1; % Negative elevator step input          

% Simulate the response of the system using lsim
[y_ph, t_out, x_ph] = lsim(sys_long_full, u, t, ph_init_cond);
%y_ph = step(sys_long_full)

% Plot V_t (Speed)
figure;
plot(t_out, x_ph(:, 1), 'b', 'LineWidth', 1.5);
title('Phugoid Time Response: V_t (Speed)');
xlabel('Time (s)');
ylabel('V_t');
grid on;

% Plot \alpha (Angle of Attack)
figure;
plot(t_out, x_ph(:, 2), 'r', 'LineWidth', 1.5);
title('Phugoid Time Response: \alpha (Angle of Attack)');
xlabel('Time (s)');
ylabel('\alpha (deg)');
grid on;

% Plot \theta (Pitch Angle)
figure;
plot(t_out, x_ph(:, 3), 'g', 'LineWidth', 1.5);
title('Phugoid Time Response: \theta (Pitch Angle)');
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;

% Plot q (Pitch Rate)
figure;
plot(t_out, x_ph(:, 4), 'm', 'LineWidth', 1.5);
title('Phugoid Time Response: q (Pitch Rate)');
xlabel('Time (s)');
ylabel('q (deg/s)');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Eigenmotions
%%%%%%%%%%%%%%%%%%%%%%%

C_ac_lat = C_lat([1 2], [1 2 3 4]);
D_ac_lat = zeros(2, 2);

% Full longitudinal state-space model (4 states: Vt, alpha, theta, q)
sys_lat_full = ss(A_ac_lat, B_ac_lat, C_ac_lat, D_ac_lat);

% Get the eigenvector corresponding to the short-period mode
[lat_eigvecs, ~] = eig(A_ac_lat);
%[num_states, ~] = size(sys_lat_full.A);
%disp(['Number of states: ', num2str(num_states)]);


%%%%%%%%%%%%%
%% Dutch Roll
%%%%%%%%%%%%%
dr_init_cond = real(lat_eigvecs(:, 1));
%dr_init_cond = [0,0,0,0];

% Create the time vector for simulation
t = 0:0.01:20;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for aileron and rudder
%u(:, 1) = 0.01;
u(:, 2) = -0.01; % Negative rudder step input

% Simulate the response of the system using lsim
[y_dr, t_out, x_dr] = lsim(sys_lat_full, u, t, dr_init_cond);

% Plot \beta
figure;
plot(t_out, x_dr(:, 1), 'b', 'LineWidth', 1.5);
title('Dutch Roll Time Response: \beta ');
xlabel('Time (s)');
ylabel('\beta (deg)');
grid on;

% Plot \phi 
figure;
plot(t_out, x_dr(:, 2), 'r', 'LineWidth', 1.5);
title('Dutch Roll Time Response: \phi');
xlabel('Time (s)');
ylabel('\phi (deg)');
grid on;

% Plot p (Roll Rate)
figure;
plot(t_out, x_dr(:, 3), 'g', 'LineWidth', 1.5);
title('Dutch Roll Time Response: p (Roll rate)');
xlabel('Time (s)');
ylabel('p (deg/s)');
grid on;

% Plot r (Yaw Rate)
figure;
plot(t_out, x_dr(:, 4), 'm', 'LineWidth', 1.5);
title('Dutch Roll Time Response: r (Yaw Rate)');
xlabel('Time (s)');
ylabel('r (deg/s)');
grid on;

%%%%%%%%%%%%%
%% A-periodic
%%%%%%%%%%%%%


ar_init_cond = lat_eigvecs(:, 3);
%ar_init_cond = [0,0,0,0];

% Create the time vector for simulation
t = 0:0.01:100;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for aileron and rudder
u(:, 1) = 1; % Positive aileron step input
%u(:, 2) = -0.01; % Negative rudder step input

% Simulate the response of the system using lsim
[y_ar, t_out, x_ar] = lsim(sys_lat_full, u, t, ar_init_cond);

% Plot \beta
figure;
plot(t_out, x_ar(:, 1), 'b', 'LineWidth', 1.5);
title('A-periodic Time Response: \beta ');
xlabel('Time (s)');
ylabel('\beta (deg)');
grid on;

% Plot \phi 
figure;
plot(t_out, x_ar(:, 2), 'r', 'LineWidth', 1.5);
title('A-periodic Time Response: \phi');
xlabel('Time (s)');
ylabel('\phi (deg)');
grid on;

% Plot p (Roll Rate)
figure;
plot(t_out, x_ar(:, 3), 'g', 'LineWidth', 1.5);
title('A-periodic Time Response: p (Roll rate)');
xlabel('Time (s)');
ylabel('p (deg/s)');
grid on;

% Plot r (Yaw Rate)
figure;
plot(t_out, x_ar(:, 4), 'm', 'LineWidth', 1.5);
title('A-periodic Time Response: r (Yaw Rate)');
xlabel('Time (s)');
ylabel('r (deg/s)');
grid on;


%%%%%%%%%%%%%
%% Spiral
%%%%%%%%%%%%%

spiral_init_cond = lat_eigvecs(:, 4);
%dr_init_cond = [0,0,0,0];

% Create the time vector for simulation
t = 0:0.01:50;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for aileron and rudder
u(:, 1) = 1; % Negative aileron step input
%u(:, 2) = -0.01; % Negative rudder step input

% Simulate the response of the system using lsim
[y_spiral, t_out, x_spiral] = lsim(sys_lat_full, u, t, spiral_init_cond);

% Plot \beta
figure;
plot(t_out, x_spiral(:, 1), 'b', 'LineWidth', 1.5);
title('Spiral Time Response: \beta ');
xlabel('Time (s)');
ylabel('\beta (deg)');
grid on;

% Plot \phi 
figure;
plot(t_out, x_spiral(:, 2), 'r', 'LineWidth', 1.5);
title('Spiral Time Response: \phi');
xlabel('Time (s)');
ylabel('\phi (deg)');
grid on;

% Plot p (Roll Rate)
figure;
plot(t_out, x_spiral(:, 3), 'g', 'LineWidth', 1.5);
title('Spiral Time Response: p (Roll rate)');
xlabel('Time (s)');
ylabel('p (deg/s)');
grid on;

% Plot r (Yaw Rate)
figure;
plot(t_out, x_spiral(:, 4), 'm', 'LineWidth', 1.5);
title('Spiral Time Response: r (Yaw Rate)');
xlabel('Time (s)');
ylabel('r (deg/s)');
grid on;




