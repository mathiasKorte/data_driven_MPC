%% Prepare workspace
close all;
clear all;
clc;

%% Create the state-space representatnion
noise = 0.01;

% State-space system matrices
A=[-1.2,0.03;0.075,-1];
B=[2;0];
C=[0,1];
D=0;

% Matrix dimensions
x_dim = 2;
y_dim = 1;
u_dim = 1;

%% Generate input and output trajectories
u_d = [];
y_d = [];

% Generate random start state
x = randn(x_dim,1);

for i=1:50
    u = randn(u_dim,1);
    y = C * x + D * u + randn(y_dim,1)*noise;
    x = A * x + B * u;
    u_d = [u_d; u'];
    y_d = [y_d; y'];
end

%% Open-loop plots

figure
subplot(2,1,1)
plot(y_d)
grid on;
title('Y');
subplot(2,1,2)
plot(u_d)
grid on;
title('U')

%% Initialization of DDMPC
n = x_dim;      % Upper bound on system order   
L = 10;         % Prediction horizon
R = 0;          % Cost matrix R
Q = eye(y_dim); % Cost matrix Q

ddmpc= DDMPC(u_d,y_d,Q,R,n,L,'y_s',0,....
    'G_mat_u',[1;-1],'g_vec_u',[1;1]*5,....
    ...%'G_mat_y',[1,0;-1,0;0,1;0,-1],'g_vec_y',ones(4,1)*30,....
    'lambda_sigma',100, 'ctrl_mode', 'robust');
    
u = randn(u_dim,1)*0;
x = randn(x_dim,1)*10;

%% Run DDMPC

u_traj = [];
y_traj =[];
for i=1:30
    y = C * x + D * u;
    x = A * x + B * u;

     y = y+randn(y_dim,1)*noise;

    u = ddmpc.step(u,y);
    u_traj = [u_traj; u'];
    y_traj = [y_traj; y'];
end

%% Closed-loop plots
% Plot closed-loop input
figure()
hold on;
title('');
xlabel('Time t');
ylabel('Closed-loop input u_{app} [V]'); 
plot(u_traj,'DisplayName','Input u_1');   % Trajectory of input 1
grid on;
legend('Location','southeast');

% Plot closed-loop output
figure
hold on;
title('');
xlabel('Time t');
ylabel('Closed-loop output \omega [s^{-1}]');
plot(y_traj,'DisplayName','Output y_1');   % Trajectory of tank level 1
grid on;
legend('Location','southeast');