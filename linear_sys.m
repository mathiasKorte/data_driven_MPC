%% Prepare workspace
close;
clear;
clc;

%% Create the system
% The system has two states and one single input
A = [1, 0.9;0,1.1]*1.1;
B = [1.3,2; 0,1];
C = [1.15,2.3;5,6];
D = [0,1;2,3];

sys = ss(A,B,C,D,0.05);

x0 = [-0.2 0.3];                % Initial system states
t = 0:0.05:8;                   % Lenth of the trajectory (samplingtime 0.05s)
u_d = zeros(length(t),2);       % Create input trajectory
u_d(t>=2,1) = 1;
u_d(t>=2,2) = 2;
u_d = randn(length(t),2);

[y_d,t] = lsim(sys,u_d,t,x0);   % Simulate system response

figure()
lsim(sys,u_d,t,x0)              % Plot system response
legend('System')
grid on

%% Initalize DDMPC
n = 2;      % Upper system order
L = 50;     % Prediction horizon
R = 0.1*eye(2);    % Cost matrix R
Q = 1*eye(2);      % Cost matix Q

ddmpc= DDMPC(u_d,y_d,Q,R,1,L);

%% Training
% Initial training values
u = randn(2,1);
x = randn(n,1);
y_traj =[];
for i=1:100
    y = C * x + D * u;
    x = A * x + B * u;
    u = ddmpc.step(u,y); 
    y_traj = [y_traj; y'];
end
figure
plot(y_traj)