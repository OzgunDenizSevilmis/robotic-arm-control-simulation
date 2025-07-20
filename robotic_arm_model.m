% Robot Arm Control
clear; clc;

%% System Parameters
L1 = 1.0; % Length of the first link (meters)
L2 = 0.75; % Length of the second link (meters)
J1 = 0.01; % Inertia of joint 1 (kg.m^2)
b1 = 0.1;  % Damping coefficient of joint 1 (N.m.s/rad)
J2 = 0.005; % Inertia of joint 2 (kg.m^2)
b2 = 0.08;  % Damping coefficient of joint 2 (N.m.s/rad)

% PID Controller Gains
Kp1 = 15; Ki1 = 2; Kd1 = 5; % Joint 1 PID gains
Kp2 = 12; Ki2 = 1.5; Kd2 = 4; % Joint 2 PID gains

% Desired Joint Angles (in radians)
theta1_desired = pi/6; % Target angle for joint 1 (30 degrees)
theta2_desired = pi/4; % Target angle for joint 2 (45 degrees)

% Simulation Parameters
sim_time = 50; % Total simulation time (seconds)
dt = 0.01; % Time step (seconds)

%% Simulation Variables
theta1 = 0; theta2 = 0;
theta1_dot = 0; theta2_dot = 0;
int_error1 = 0; int_error2 = 0;
prev_error1 = 0; prev_error2 = 0;

time = 0:dt:sim_time;
theta1_history = zeros(size(time));
theta2_history = zeros(size(time));
x_history = zeros(size(time));
y_history = zeros(size(time));

%% Simulation Loop
for i = 1:length(time)
    % Error Calculations
    error1 = theta1_desired - theta1;
    error2 = theta2_desired - theta2;

    % PID Calculations
    int_error1 = int_error1 + error1 * dt;
    int_error2 = int_error2 + error2 * dt;

    der_error1 = (error1 - prev_error1) / dt;
    der_error2 = (error2 - prev_error2) / dt;

    % Control Torques
    torque1 = Kp1 * error1 + Ki1 * int_error1 + Kd1 * der_error1;
    torque2 = Kp2 * error2 + Ki2 * int_error2 + Kd2 * der_error2;

    % Dynamics Equations
    alpha1 = (torque1 - b1 * theta1_dot) / J1;
    alpha2 = (torque2 - b2 * theta2_dot) / J2;

    % Update Velocities and Positions
    theta1_dot = theta1_dot + alpha1 * dt;
    theta2_dot = theta2_dot + alpha2 * dt;
    theta1 = theta1 + theta1_dot * dt;
    theta2 = theta2 + theta2_dot * dt;

    % Save End-Effector Position Only
    x_history(i) = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y_history(i) = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    % Update Previous Errors
    prev_error1 = error1;
    prev_error2 = error2;
end
%% Visualization: End-Effector Trajectory
figure;
plot(x_history, y_history, 'm', 'LineWidth', 1.5); hold on;
scatter(x_history(1), y_history(1), 'go', 'filled'); % Start position
scatter(x_history(end), y_history(end), 'ro', 'filled'); % End position
title('End-Effector Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Trajectory', 'Start Position', 'End Position');
grid on;

%% Open-Loop and Closed-Loop Analysis
s = tf('s');
G1 = 1 / (J1 * s^2 + b1 * s); % Open-loop transfer function for Joint 1
G2 = 1 / (J2 * s^2 + b2 * s); % Open-loop transfer function for Joint 2

PID1 = Kp1 + Ki1/s + Kd1*s;
PID2 = Kp2 + Ki2/s + Kd2*s;

CL1 = feedback(PID1 * G1, 1); % Joint 1 closed-loop system
CL2 = feedback(PID2 * G2, 1); % Joint 2 closed-loop system

figure;
subplot(2, 1, 1);
step(G1, G2, sim_time);
title('Open-Loop Step Response');
xlabel('Time (s)');
ylabel('Output');
legend('Joint 1', 'Joint 2');
grid on;

subplot(2, 1, 2);
step(CL1, CL2, sim_time);
title('Closed-Loop Step Response');
xlabel('Time (s)');
ylabel('Output');
legend('Joint 1', 'Joint 2');
grid on;
