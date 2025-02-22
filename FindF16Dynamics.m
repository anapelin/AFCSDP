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
% Set xa/gD to 0 
set_param('LIN_F16Block/Gain', 'Gain', '0');
%% Trim aircraft to desired altitude and velocity
%%
%altitude = input('Enter the altitude for the simulation (ft)  :  ');
%velocity = input('Enter the velocity for the simulation (ft/s):  ');

% Set up the altitude and velocity as they are constant for this part
altitude = 15000; % ft
velocity = 500; % ft/s


FC_flag = 1; % Trim for steady wings-level flight

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for lofi model at desired altitude and velocity
%%
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


%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [3 5 7 8 11 13 14];
long_inputs = [1 2];
long_outputs = [3 5 7 8 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);

%%%%%%%%%%%%%%%%%%%%
%% Lateral Direction
%%%%%%%%%%%%%%%%%%%%

lat_states = [4 6 7 9 10 12 13 15 16];
lat_inputs = [1 3 4];
lat_outputs = [4 6 7 9 10 12];

SS_lat_lo = ss(SS_lo.A(lat_states,lat_states), SS_lo.B(lat_states,lat_inputs), SS_lo.C(lat_outputs,lat_states), SS_lo.D(lat_outputs,lat_inputs));

SS_lat_lo.StateName = SS_lo.StateName(lat_states);

SS_lat_lo.InputName= SS_lo.InputName(lat_inputs);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get the an transfer function based on multiple transfer functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
s = tf('s');
nz_tf = tf(SS_lo(15, 2));
qdot_tf = tf(s * tf(SS_lo(11, 2)));


xa_init = 0;
xa_final = 1;

for xa = xa_init:0.1:xa_final
    xa_gD = xa / 3.280840;
    %disp("xa_gD");
    %xa_gD
    an_tf = tf(nz_tf + xa_gD * qdot_tf);
    %an_tf
    disp("Zeros");
    zero(an_tf)
    %disp("Poles");
    %pole(an_tf)
end
%}


%%%%%%%%%%%%%%%%%%%%%%%
%% Find the ss matrices
%%%%%%%%%%%%%%%%%%%%%%%

disp('Low-fidelity model dimensions:');
size_A_lo = size(SS_lo.A);
size_B_lo = size(SS_lo.B);
size_C_lo = size(SS_lo.C);
size_D_lo = size(SS_lo.D);

disp(['A matrix: ', num2str(size_A_lo)]);
disp(['B matrix: ', num2str(size_B_lo)]);
disp(['C matrix: ', num2str(size_C_lo)]);
disp(['D matrix: ', num2str(size_D_lo)]);

% Extract the 19th row from C and D matrices for normal acceleration
C_an = SS_lo.C(19, :);  % Row 19 of C matrix (for normal acceleration)
D_an = SS_lo.D(19, :);  % Row 19 of D matrix (for inputs)

% Linearized output equation for normal acceleration (an)
% an = C_an * x + D_an * u
disp('Output equation for normal acceleration (an):');
disp(['an = ', num2str(C_an), ' * x + ', num2str(D_an), ' * u']);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the elevator-to-normal-acceleration tf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set xa/gD to 0 
set_param('LIN_F16Block/Gain', 'Gain', '0');

% Select the input and output
input_index = 2;  % Elevator actuator input
output_index = 19; % Normal acceleration output


% Extract the transfer function
%elevator_to_normal_accel_tf = tf(SS_lo(output_index, input_index));
% Display the transfer function
%disp('Elevator-to-Normal-Acceleration Transfer Function:');
%elevator_to_normal_accel_tf

% Define the negative step input
step_amplitude = -1; % Negative step input (e.g., -1 degree elevator command)

% Define the simulation time
t_end = 4; % Simulate for 5 seconds
dt = 0.01; % Time step for smooth response
time = 0:dt:t_end;

% Get the elevator-to-normal-acceleration transfer function
elevator_to_normal_accel_tf = tf(SS_lo(output_index, input_index)); 
elevator_to_normal_accel_tf = minreal(elevator_to_normal_accel_tf);

% Show the zeros and poles of the transfer function
zero(elevator_to_normal_accel_tf);
pole(elevator_to_normal_accel_tf);

% Simulate the step response
[response, time] = step(step_amplitude * elevator_to_normal_accel_tf, time);

% Plot the response
figure;
plot(time, response, 'LineWidth', 1.5);
grid on;
xlabel('Time (seconds)');
ylabel('Normal Acceleration (an) [g]');
title('Normal Acceleration Response to Negative Step Elevator Command');
legend('an [xa = 0 ft]');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute different responses based on xa value, get zeros and poles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get all the values for xa/gD
gD = 32.1740;
xa_values_ft = [0, 5, 5.9, 6, 7, 15]; 
xa_gD_values = xa_values_ft / gD; % gD = 3.280840 ft/s^2

% Initialize storage for unfiltered poles and zeros
poles_zeros = struct('xa_gD', [], 'poles', [], 'zeros', []);

% Initialise storage for filtered poles and zeros
filtered_poles_zeros = struct('xa_gD', [], 'poles', [], 'zeros', []);

% Define the threshold for filtering the poles and zeros
threshold = 1e-2;

% Create the figure and plot all lines on it
figure; % Open a new figure
hold on; % Retain plots on the same figure
grid on; % Add grid lines

    
for i = 1:length(xa_gD_values)
    % Update the gain in Simulink model
    xa_gD = xa_gD_values(i);
    set_param('LIN_F16Block/Gain', 'Gain', num2str(xa_gD));

    %% Find trim for lofi model at desired altitude and velocity
    disp('Trimming Low Fidelity Model:');
    fi_flag_Simulink = 0;
    [trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

    %% Find the state space model for the lofi model at the desired alt and vel.
    trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
    operating_point = operpoint('LIN_F16Block');
    operating_point.Inputs(1).u = trim_thrust_lin;
    operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2);
    operating_point.Inputs(4).u = trim_control_lin(3);

    SS_lo = linearize('LIN_F16Block');

    % Define the negative step input
    step_amplitude = -1; % Negative step input
    t_end = 4; % Simulation end time
    dt = 0.01; % Time step
    time = 0:dt:t_end;

    % Get the transfer function
    input_index = 2; % Elevator actuator input
    output_index = 19; % Normal acceleration output
    elevator_to_normal_accel_tf = tf(SS_lo(output_index, input_index));
    elevator_to_normal_accel_tf = minreal(elevator_to_normal_accel_tf);

    % Extract original poles and zeros
    poles = pole(elevator_to_normal_accel_tf);
    zeros = zero(elevator_to_normal_accel_tf);

    % Store original poles and zeros
    poles_zeros(i).xa_gD = xa_gD;
    poles_zeros(i).poles = poles;
    poles_zeros(i).zeros = zeros;
    
    
    % Compute step responses
    [original_response, t_original] = step(step_amplitude * elevator_to_normal_accel_tf, time);
    
    % Plot the response
    plot(t_original, original_response, 'LineWidth', 1.5, 'DisplayName', ['xa = ', num2str(xa_values_ft(i)), ' ft']);

end

% Finalize the plot
xlabel('Time (seconds)');
ylabel('Normal Acceleration (an) [g]');
title('Normal Acceleration Response to Negative Step Elevator Command');
legend show; % Call legend after the loop to include all lines
legend('Location','best');

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display poles and zeros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Poles and Zeros for Each Transfer Function:');
for i = 1:length(poles_zeros)
    fprintf('xa = %.2f\n', poles_zeros(i).xa_gD * 3.280840);
    %fprintf('Poles:\n');
    %disp(poles_zeros(i).poles);
    fprintf('Zeros:\n');
    disp(poles_zeros(i).zeros);
end
%}
%}
% Reinitialise xa/gD to be 0 
set_param('LIN_F16Block/Gain', 'Gain', '0');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate the instantaneous center of rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Uncomment from here onwards because the loop will keep runnning
%{
disp("Finding the instantaneous center of rotation")
% Set xa range and step size
xa_start = 5; % Starting value of xa
xa_end = 8;   % Ending value of xa
step_size = 0.1; % Increment size for xa
gD = 32.1740; % ft/s^2

% Loop through xa values
for xa = xa_start:step_size:xa_end
    set_param('LIN_F16Block/Gain', 'Gain', num2str(xa / gD));

    %% Find trim for lofi model at desired altitude and velocity
    disp('Trimming Low Fidelity Model:');
    fi_flag_Simulink = 0;
    [trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

    %% Find the state space model for the lofi model at the desired alt and vel.
    trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
    operating_point = operpoint('LIN_F16Block');
    operating_point.Inputs(1).u = trim_thrust_lin;
    operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2);
    operating_point.Inputs(4).u = trim_control_lin(3);

    SS_lo = linearize('LIN_F16Block');

    % Get the transfer function
    input_index = 2; % Elevator actuator input
    output_index = 19; % Normal acceleration output
    elevator_to_normal_accel_tf = tf(SS_lo(output_index, input_index));
    
    % Extract zeros and poles
    fprintf('xa = %.1f\n', xa);
    zeros = zero(elevator_to_normal_accel_tf)
end

%}


