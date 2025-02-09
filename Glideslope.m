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
s = tf('s');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Trim aircraft to desired altitude and velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
altitude = 5000; %ft
velocity = 300;  %ft/s
height_runway = 3000; %ft
height_intercept = altitude - height_runway; %ft

FC_flag = 1; % Trim for steady wings-level flight

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial guess for trim
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find trim for lofi model at desired altitude and velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the state space model for the lofi model at the desired alt and vel.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block');

newline = sprintf('\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired system state names for reduced model 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Decoupled states
% Inputs of the model h,Vt,α,θ,q -> 3, 7, 8, 5, 11
% Outputs of the model h, Vt, θ, q -> 3, 7, 5, 11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

long_indices_states_reduced = [3 7 8 5 11 13 14];
long_indices_states = [3 7 8 5 11];
long_indices_input = [1 2]; % throttle and elev
long_indices_output = [3 7 8 5 11];
A_indices_states_ac = [1 2 3 4 5];
B_indices_states_ac = [6 7];


trim_state = trim_state_lin([long_indices_states]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Declare space state matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_full = SS_lo.A;
B_full = SS_lo.B;
C_full = SS_lo.C;
D_full = SS_lo.D;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract longitudinal dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_reduced = A_full(long_indices_states_reduced, long_indices_states_reduced);
A_long = A_reduced(A_indices_states_ac, A_indices_states_ac);
B_long = A_reduced(A_indices_states_ac, B_indices_states_ac);
%A_long = A_full(long_indices_states, long_indices_states); % 5x5 matrix
%B_long = B_full(long_indices_states, long_indices_input); % 5x2 matrix  
C_long = C_full(long_indices_output, long_indices_states);  % 5x5 matrix    
D_long = D_full(long_indices_output, long_indices_input); % 5x2 matrix


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct the reduced model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
reducedModel = ss(A_long, B_long, C_long, D_long);

% Check the matrix A
disp("Reduced state space matrix eigs");

% Check the eigenvalues 
eigs_long = eig(A_long);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define transfer function for engine and elevator to build subsystem block
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H_engine = tf(1, [1 1]);
H_elevator = tf(20.2, [1 20.2]);
% Saturations for engine and elevator
engine_saturation = [1000, 19000]; % lbs
elevator_saturation = [-25, 25]; % degree
K_elevator = 180/pi; %transfer input from rad to deg

% Glide path angle
glideslope_angle_deg = 3; % deg


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Finding the runway position for flare
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runway_location = height_intercept/tan(glideslope_angle_deg * pi / 180) + velocity * 10;
disp("Runway location is at: ");
disp(runway_location); % ft


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Check the trim condition/overwrite the level flight condition otherwise 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trim_condition = trim_state_lo(long_indices_states); % h,Vt,α,θ,q
%trim_condition = [2000; 300; 0; 0; 0]; % if not the desired trim condition

% Making the reduced tf with MIMO 2 in 5 out --->> TBD if 4 outputs or 5 are needed
model_tf = tf(reducedModel);
model_tf


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Design the glideslope controller, starting with the AFCS + Airplane block
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define the inner loop
Kq = 1;
tf_elevator_q = model_tf(5,2);
tf_elevator_q = tf_elevator_q;
open_inner_loop = Kq * tf_elevator_q;
inner_loop = feedback(open_inner_loop, 1);

%Testing the transfer functions

% Tune inner loop 
figure
rlocus(open_inner_loop)
title('Root Locus Diagram Inner Loop');
xlabel('Real Axis');
ylabel('Imaginary Axis');
%xlim([0,10]);

% Define complete Airplane + AFCS block
Ktheta = 1;
outer_loop = Ktheta * inner_loop * 1/s;
outer_loop = feedback(outer_loop,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute the transfer function from theta to the flight path angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf_elevator_alpha = model_tf(3,2);
tf_elevator_theta = model_tf(4,2);
tf_theta_gamma = 1 - tf_elevator_alpha / tf_elevator_theta;

figure
rlocus(tf_elevator_alpha);
%figure
%pzmap(tf_elevator_theta);

%figure
%rlocus(tf_elevator_theta);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Assemble all elements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kcoupler = 1000;
W1 = 0.1;
H_receiver = 1;
H_coupler = Kcoupler * (1 + W1/s);
H_airplaneAFCS = outer_loop;
H_thetagamma = tf_theta_gamma;
% overwrite theta gamma bc broken
H_thetagamma.Numerator{1} = [1,-3.48656642887370,-28.1495922105852,0.251293103322596];
H_thetagamma.Denominator{1} = [1,47.5870761349079,1.48460909372540,1.19351731843969];


% Initialize slant range from the distance to the runway and the altitude
R = sqrt(runway_location * runway_location + height_intercept * height_intercept); % TBD slant range 
H_glideslopedynamics = tf(velocity/s * pi /180);
H_range = 1/R;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% Define simulation parameters
simulationTime = 60; % Total simulation duration in seconds
interceptTime = 10;  % Time after which interception should occur
interceptAltitude = height_intercept; % ft, initial altitude for interception
interceptVelocity = velocity; % ft/s, initial velocity

% Initialize simulation 
t = 0:0.01:simulationTime;

trim_condition(3) = trim_condition(3) * 180/pi;
trim_condition(4) = trim_condition(4) * 180/pi;

latex_A = latex(sym(A_long))
latex_B = latex(sym(B_long))
latex_C = latex(sym(C_long))




