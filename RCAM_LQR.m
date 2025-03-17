clc
clear
close all

%% Add Path of "tools" directory
% Get the current file's directory
currentDir = fileparts(mfilename('fullpath'));

% Go to tools directory
parentDir = fullfile(currentDir, './tools');

% Add the parent directory to the MATLAB path
addpath(parentDir);

%% Define Trim Condition
Z_guess = zeros(17, 1);
Z_guess(1) = 85;

RCAM_LQR_Trim

%% Linearize about the new trim condition
linearize_trim

%% Determine control gain, K

%% Split into longitudinal and lateral/directional
decoupled_states

A_long = A_tilde(1:4,1:4);
A_lat = A_tilde(5:8,5:8);

B_long = B_tilde(1:4, :);
B_lat = B_tilde(5:8, :);

% define Q and R
%% Uncomment for a controller with disconnected Longitudinal and Lateral/Direction Control
Q = eye(4)*10;
R = eye(5)*1;

[K_long, ~, ~] = lqr(A_long, B_long, Q, R);
[K_lat, ~, ~] = lqr(A_lat, B_lat, Q, R);

K = [K_long K_lat, zeros(5,1)];
K = K * inv(T);

%% Uncomment for a controller that does not disconnect Longintudinal and Lateral/Directional Control

% Q = eye(9)*10;
% R = eye(5)*0.8;
% 
% [K, S, E] = lqr(A, B, Q, R);

disp("K: ")
disp(K)

%% Run the Sim
% define initial sim parameters
TF = 30;
x0 = X_bar;% + [1;1;1;0;0;0;3;2;1]*0.1;   % initial state of RCAM_model
P0 = [0;0;7000];                       % initial navigation position
GEO0 = [47.531478*pi/180; -122.303680*pi/180; 700];                   % initial position in Geodetic Coordinates

% define disturbance parameters (in degrees)
ail_dist = 25;      % deg
stab_dist = -30;    % deg
rud_dist = -30;      % deg
dist_dt = 5;        % sec

% run the sim
simOut = sim('RCAMSimulation.slx');

%% Plot the results

simX = simOut.simX;
simU = simOut.simU;

t = simX.Time;

u1 = simU.Data(:, 1);
u2 = simU.Data(:, 2);
u3 = simU.Data(:, 3);
u4 = simU.Data(:, 4);
u5 = simU.Data(:, 5);

x1 = simX.Data(:, 1);
x2 = simX.Data(:, 2);
x3 = simX.Data(:, 3);
x4 = simX.Data(:, 4);
x5 = simX.Data(:, 5);
x6 = simX.Data(:, 6);
x7 = simX.Data(:, 7);
x8 = simX.Data(:, 8);
x9 = simX.Data(:, 9);

% Plot controls
figure
subplot(5,1,1)
plot(t, u1, t, U_bar(1)*ones(size(t)), 'r:')
legend('u_1', 'u_1 bar')
grid on

subplot(5,1,2)
plot(t, u2, t, U_bar(2)*ones(size(t)), 'r:')
legend('u_2', 'u_2 bar')
grid on

subplot(5,1,3)
plot(t, u3, t, U_bar(3)*ones(size(t)), 'r:')
legend('u_3', 'u_3 bar')
grid on

subplot(5,1,4)
plot(t, u4, t, U_bar(4)*ones(size(t)), 'r:')
legend('u_4', 'u_4 bar')
grid on

subplot(5,1,5)
plot(t, u5, t, U_bar(5)*ones(size(t)), 'r:')
legend('u_5', 'u_5 bar')
grid on

% Plot states
figure
subplot(3,3,1)
plot(t, x1, t, X_bar(1)*ones(size(t)), 'r:')
legend('x_1', 'x_1 bar')
grid on

subplot(3,3,4)
plot(t, x2, t, X_bar(2)*ones(size(t)), 'r:')
legend('x_2', 'x_2 bar')
grid on

subplot(3,3,7)
plot(t, x3, t, X_bar(3)*ones(size(t)), 'r:')
legend('x_3', 'x_3 bar')
grid on

subplot(3,3,2)
plot(t, x4, t, X_bar(4)*ones(size(t)), 'r:')
legend('x_4', 'x_4 bar')
grid on

subplot(3,3,5)
plot(t, x5, t, X_bar(5)*ones(size(t)), 'r:')
legend('x_5', 'x_5 bar')
grid on

subplot(3,3,8)
plot(t, x6, t, X_bar(6)*ones(size(t)), 'r:')
legend('x_6', 'x_6 bar')
grid on

subplot(3,3,3)
plot(t, x7, t, X_bar(7)*ones(size(t)), 'r:')
legend('x_7', 'x_7 bar')
grid on

subplot(3,3,6)
plot(t, x8, t, X_bar(8)*ones(size(t)), 'r:')
legend('x_8', 'x_8 bar')
grid on

subplot(3,3,9)
plot(t, x9, t, X_bar(9)*ones(size(t)), 'r:')
legend('x_9', 'x_9 bar')
grid on
