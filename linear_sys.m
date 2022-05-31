%% Prepare workspace
close;
clear;
clc;

%% Create the system
% The system has two states and one single input
A = [-1.5 -3; 3 -1];
B = [1.3; 0];
C = [1.15 2.3];
D = 0;

sys = ss(A,B,C,D);

x0 = [-0.2 0.3];                % Initial system states
t = 0:0.05:8;                   % Lenth of the trajectory (samplingtime 0.05s)
u_d = zeros(length(t),1);       % Create input trajectory
u_d(t>=2) = 1;

[y_d,t] = lsim(sys,u_d,t,x0);   % Simulate system response

figure()
lsim(sys,u_d,t,x0)              % Plot system response
legend('System')
grid on

%% Initalize DDMPC
n = 2;      % Upper system order
L = 10;     % Prediction horizon
R = 0.1;    % Cost matrix R
Q = 1;      % Cost matix Q

ddmpc= DDMPC(u_d,y_d,Q,R,n,L);

%% Training
% Initial training values
u = 10;
x = ones(n,1);

for i=1:100
    y = C * x + D * u;
    x = A * x + B * u;
    u = ddmpc.step(u,y);
    [y,u]
end