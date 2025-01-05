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
close all

rng(1,"twister");

%% Trim aircraft to desired altitude and velocity
%%
altitude = 40000;
velocity = 300; % ft/s
g = 32.1740; %ft/s^2

FC_flag = 1; % Trim for steady wings-level flight

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 0;              % AOA, degrees
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

long_indices_states = [7 8 5 11 14 13];
long_indices_states_wo = [1 2 3 4];
long_indices_input = [2 1];
long_indices_output = [8 11];

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

% indices of the new matrices
indices_states = [1 2 3 4];
indices_outputs = [5 6];

A_ac_long = A_long(long_indices_states_wo, long_indices_states_wo);
B_ac_long = A_long(long_indices_states_wo, indices_outputs);
C_ac_long = C_long([1 2], [1 2 3 4]);
D_ac_long = zeros(2, 2);

%% Finding the eigenvalues 
% Longitudinal eigenvalues
eigs_full = eig(A_ac_long);
sp_eig = eigs_full([1 2]);

%% Calculate Parameters for each eigen motion
% Short period
sp_nf = abs(sp_eig(1));
sp_damp = - real(sp_eig(1))/sp_nf;
sp_p = 2*pi/abs(imag(sp_eig(1)));
sp_t12 = log(0.5)/real(sp_eig(1));


% Define the short period matrices for the reduced model
A_sp = A_ac_long([2 4], [2 4]);
B_sp = B_ac_long([2 4], [1]);
C_sp = C_ac_long([1 2], [2 4]);
D_sp = D_ac_long(:, [1]);


% Create the state space system for the short period reduced model
sys_sp_reduced = ss(A_sp, B_sp, C_sp, D_sp);
% Full longitudinal state-space model (4 states: Vt, alpha, theta, q)
sys_sp_full = ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);

% Define the eigenvectors
[full_eigvecs, ~] = eig(A_ac_long);
[reduced_eigvecs, ~] = eig(A_sp);

% Plot poles and zeros for the full longitudinal model
figure;
pzmap(sys_sp_full);
title('Pole-Zero Map: Full Short-Period Model');
grid on;

% Plot poles and zeros for the reduced short-period model
%figure;
%pzmap(sys_sp_reduced);
%title('Pole-Zero Map: Reduced Short-Period Model');
%grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Short Period Plots for q and alpha
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Choose the eigenvector corresponding to the short-period mode
sp_full_init_cond = real(full_eigvecs(:, 1)); % Short-period eigenvector (real part)
sp_reduced_init_cond = real(reduced_eigvecs(:, 2));

%sp_init_cond = [0,0,0,0];

% Create the time vector for simulation
t = 0:0.01:500;

% Define input as zero for the short period motion (no external input)
u = zeros(length(t), 2); % Two input channels for elevator and throttle 
u(:, 1) = -1; % Negative elevator step input
u_sp = ones(length(t),1);
u_sp =-1 * u_sp;

% Simulate the response of the system using lsim
[y_full_sp, t_out, x_full_sp] = lsim(sys_sp_full, u, t, sp_full_init_cond);
[y_reduced_sp, t_out, x_reduced_sp] = lsim(sys_sp_reduced, u_sp, t, sp_reduced_init_cond);


% Plot q (Pitch Rate) for both cases on the same figure
figure;
plot(t_out, x_full_sp(:, 4), 'm', 'LineWidth', 1.5); % Full model response
hold on; % Keep the current plot
plot(t_out, x_reduced_sp(:, 2), 'g', 'LineWidth', 1.5); % Reduced model response

% Add title and axis labels
title('Short Period Time Response: q (Pitch Rate)');
xlabel('Time (s)');
ylabel('q (deg/s)');

% Add grid
grid on;

% Add legend
legend('Full Model', 'Reduced Model', 'Location', 'Best');

% Release hold
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate the requirements for nf, T_theta_2, damping ratio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nf_req = 0.03 * 0.3048 * velocity; % 0.3048 from ft/s to m/s 
t_theta_2_req = 1 / (0.75 * nf_req);
damp_ratio_req = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate transfer function from elevator to alpha and q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%outputs: alpha, q
%inputs: elevator, thrust
%states: alpha, q
%%% Ignore actuator dynamics

%{
% Define transfer function for the reduced short-period model
alpha_q_tf = tf(sys_sp_reduced);

% Extract the transfer function from elevator input to alpha and q
elevator_to_alpha = alpha_q_tf(1, 1); % Output 1 (alpha) from input 1 (elevator)
elevator_to_q = alpha_q_tf(2, 1);     % Output 2 (q) from input 1 (elevator)

combined_alpha_q_tf = [elevator_to_alpha; elevator_to_q];
%}

%% Find Kalpha
tfs_sys_sp_reduced = tf(sys_sp_reduced);
tfalpha = tfs_sys_sp_reduced(1);

figure;
rlocus(-tfalpha)
xlim([-10 10])
ylim([-10 10])

%% Find Kq
Kalpha = -7.63;
A_sp_w_gain = A_sp - B_sp * C_sp(1,:) * Kalpha;
sys_sp_reduced_w_Kalpha = ss(A_sp_w_gain, B_sp, C_sp, D_sp);
tfs_sys_sp_reduced_tuned = tf(sys_sp_reduced_w_Kalpha);
tfq = tfs_sys_sp_reduced_tuned(2);

figure
rlocus(-tfq)
xlim([-4 4])
ylim([-10 10])


%% Check for the step response of the closed loop transfer function
Kq = -2.31;
CL_tf = feedback(Kq*tfq, 1);

% Plot the poles and zeros of the closed loop transfer function of q
figure
pzmap(CL_tf)

% Plot the step response of the closed loop transfer function of q
figure;
step(CL_tf)
xlim([0 10])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Add leadlag filter to tune for t_theta_2 required
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define parameters for the lead-lag filter
Kfilter = 1;      % Gain
T1 = 1.1;   % Lead time constant (seconds)
T2 = 4.1788;   % Lag time constant (seconds)

% Define the transfer function of the lead-lag filter
numerator = Kfilter * [T1, 1];       % Coefficients for T1*s + 1
denominator = [2.267, 0.5426];         % Coefficients for T2*s + 1
leadlag_filter = tf(numerator, denominator);

% Calculate tf of q with the lead lad filter applied outside the loop
tfq_tuned = leadlag_filter * CL_tf;
tfq_tuned = minreal(tfq_tuned);

% Find the parameters T_theta_2, wn and zeta from the transfer function
[numerator, denominator]= tfdata(tfq_tuned);
numerator = numerator{1};
denominator = denominator{1};

disp("The coefficients in the numerator are: ");
disp(numerator);
disp(denominator);

% Calculate natural frequency, t_theta_2 and damping ratio
t_theta_2 = -numerator(2)/Kq; % sec 
wn = sqrt(denominator(3)); % rad/s
zeta = denominator(2)/wn/2; % unitless

disp("The time to double amplitude of theta is: ");
disp(t_theta_2);

disp("The natural frequency is: ");
disp(wn);

disp("The damping constant is: ");
disp(zeta);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find K_masa such that tuned q_ff convereges to q_input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Define K_masa
K_masa = 1 / 0.14 / 0.88;

% Define the time vector for simulation
t = 0:0.01:10;

% Define the pitch rate command (q_cmd)
q_cmd = ones(length(t)); % Step input of 1 deg/s

% Simulate the system with feedback and feedforward
u_ff = ones(length(t), 1) * K_masa; % Feedforward input
[y_tuned, t_out, x_tuned] = lsim(tfq_tuned, u_ff, t);

% Plot the pitch rate response
figure;
plot(t_out, y_tuned, 'b', 'LineWidth', 1.5); % Pitch rate response (q)
hold on;
plot(t, q_cmd, 'r--', 'LineWidth', 1.5); % Commanded pitch rate
title('Pitch Rate Response with Feedforward Gain');
xlabel('Time (s)');
ylabel('Pitch Rate (q, deg/s)');
legend('Pitch Rate (q)', 'Commanded Pitch Rate (q_{cmd})');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Verify the requirements are met for CAP and Gibson based on found natural frequency, t_theta_2 and damping ratio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CAP_value = wn * wn/velocity * g * t_theta_2;
DBqss = t_theta_2 - 2 * zeta/wn;

disp("Requirements: ");
disp("CAP criterion: ");
disp(CAP_value);
disp("Gibson criterion: ");
disp(DBqss);