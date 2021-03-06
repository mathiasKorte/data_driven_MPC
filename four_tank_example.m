%% Prepare workspace
close all;
clear all;
clc;

%% Create the state-space representatnion
ts = 1.5;       % Sampling time 1.5s
x = [0 0 0 0]; % Define x_0

% Generate a input trajectory
t = 0:ts:225;

% Generate a random two dimensional input [0,60]
min = 20;
max = 30;
u = (max-min).*rand(length(t),2) + min;

% Generate the corresponding system response to the random input
y = [];
for i=1:length(u)
    x = fourTankStep(x,u(i,:),ts);
    y = [y; [x(1) x(2)]];
end

%% Load input and output data from file
% Load files "data/four_tank_input.mat", "data/four_tank_output.mat" and 
% "data/four_tank_state.mat"
% u = load('data\four_tank_input','-mat').u;
% y = load('data\four_tank_output.mat','-mat').y;
% x = load('data\four_tank_state.mat','-mat').x;
%% Open-loop plots

% Plot open-loop input
figure
hold on;
title('');
xlabel('Time t');
ylabel('Open-loop input [cm^3/s]'); 
plot(u(:,1),'DisplayName','Input u_1');   % Trajectory of input 1
plot(u(:,2),'DisplayName','Input u_2');   % Trajectory of input 2
grid on;
legend;

% Plot open-loop output
figure
hold on;
title('');
xlabel('Time t');
ylabel('Open-loop input [cm]');
plot(y(:,1),'DisplayName','Output y_1');   % Trajectory of tank level 1
plot(y(:,2),'DisplayName','Output y_2');   % Trajectory of tank level 2
grid on;
legend;

%% Initialization of DDMPC
n = 4;                  % Upper bound on system order
N = 150;                % Input trajectory length
L = 35;                 % Prediction horizon
Q = eye(2);             % Cost matrix Q
R = 0.00001*eye(2);           % Cost matrix R
lambda_alpha = 5e-5;    % Regularization parameter lambda_alpha
lambda_sigma = 2e5;     % Regularization parameter lambda_sigma

ddmpc = DDMPC(u,y,Q,R,n,L, ...
     'y_s',[15,15], ...
     'G_mat_u',[eye(2);-eye(2)], 'g_vec_u',[60;60;0;0], ...
     'lambda_alpha', lambda_alpha, ...
     'lambda_sigma',lambda_sigma, ...
     'ctrl_mode', 'nonlinear');

%% Run DDMPC

for i=1:100
    u_new = u(end,:);
    y_new = y(end,:);
    
    u_new = ddmpc.step(u_new',y_new');
    u_new = u_new';

    x = fourTankStep(x,u_new,ts);
    y_new = [x(1) x(2)];
    
    u = [u;u_new];
    y = [y;y_new];
end

%% Closed-loop plots

% Plot closed-loop input
figure
hold on;
title('');
xlabel('Time t');
ylabel('Closed-loop input [cm^3/s]'); 
plot(u(:,1),'DisplayName','Input u_1');   % Trajectory of input 1
plot(u(:,2),'DisplayName','Input u_2');   % Trajectory of input 2
grid on;
legend('Location','southeast');

% Plot closed-loop output
figure
hold on;
title('');
xlabel('Time t');
ylabel('Closed-loop input [cm]');
yline(15,'-.b');
plot(y(:,1),'DisplayName','Output y_1');   % Trajectory of tank level 1
plot(y(:,2),'DisplayName','Output y_2');   % Trajectory of tank level 2
grid on;
legend('Location','southeast');

%% Function definition
function x_next = fourTankStep(x, u, ts)
    % Constructing a State-Space Model of the 4 tank system

    % Enter the values to specify the model
    A1 = 50.27; % Area of tank 1: 50.27cm2
    A2 = A1;    % Area of tank 2: 50.27cm2
    A3 = 28.27; % Area of tank 3: 28.27cm2
    A4 = A3;    % Area of tank 4: 28.27cm2
    
    a1 = 0.233; % Area of the pipe flowing out of tank 1
    a2 = 0.242; % Area of the pipe flowing out of tank 2
    a3 = 0.127; % Area of the pipe flowing out of tank 3
    a4 = a3;    % Area of the pipe flowing out of tank 4
    
    g = 981;            % Gravitational acceleration 981cm2/s
    gamma1 = 0.4;       % Ratio of water diverted to tank 1 rather than tank 3
    gamma2 = gamma1;    % Ratio of water diverted to tank 2 rather than tank 4
    
    x_dot = x;
    x(x<0)=0;

    x_dot(1) = -a1/A1 * sqrt(2*g*x(1)) + a3/A1 * sqrt(2*g*x(3)) + gamma1/A1 * u(1);
    x_dot(2) = -a2/A2 * sqrt(2*g*x(2)) + a4/A2 * sqrt(2*g*x(4)) + gamma2/A2 * u(2);
    x_dot(3) = -a3/A3 * sqrt(2*g*x(3)) + (1-gamma2)/A3 * u(2);
    x_dot(4) = -a4/A4 * sqrt(2*g*x(4)) + (1-gamma1)/A4 * u(1);

    x_next = x + ts*x_dot;
end






