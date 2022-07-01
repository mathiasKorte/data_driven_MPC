%% Prepare workspace
close;
clear;
clc;


%% Constructing a State-Space Model of the DC Motor
% Enter the values to specify the DC motor
R= 0.4;     % Ohms
L= 0.5;     % Henrys
Km = .015;  % torque constant
Kb = .015;  % emf constant
Kf = 0.2;   % Nms
J= 0.2;    % kg.m^2

% Given the values of the DC motor, we can create the state-space
% representatnion
A = [-R/L -Kb/L; Km/J -Kf/J];
B = [1/L; 0];
C = [0 1];
D = [0];

ts = 0.05;  % System sample time
sys_dc = ss(A,B,C,D,ts) % Create the discrete system

% Generate a input trajectory
t = 0:ts:8;  
u_d = randn(length(t),1);   % Generate a random one dimensional input
save('DC_motor_input.mat','u_d');
%u_d = load('DC_motor_input.mat').u_d;    % Load predefinded input for testing

%% Plot the system response.
figure()
lsim(sys_dc,u_d,t)
legend('System')
grid on

x0 = [1 2];
y_d = lsim(sys_dc,u_d,t,x0);   % Simulate system response

%% Initialization of DDMPC
n = 2;      % Upper system order
L = 5;     % Prediction horizon
R = 0.1;    % Cost matrix R
Q = 1;      % Cost matix Q

ddmpc= DDMPC(u_d,y_d,Q,R,2,L);

%% Execution of DDMPC
% Initial training values
u = randn(1,1);
x = randn(n,1);

u_traj = [];
y_traj =[];
for i=1:100
    y = C * x + D * u;
    x = A * x + B * u;
    u = ddmpc.step(u,y);
    u_traj = [u_traj; u'];
    y_traj = [y_traj; y'];
end

% Plot training
figure
plot(y_traj)
hold on;
plot(u_traj)
grid on;
legend('Output','Input');