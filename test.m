clear all
close all
clc


n = 3;
l = 10;
Q = 1;
R = 0.001;

u_traj = [];
y_traj = [];
x_dim = 3;
x = zeros(x_dim,1);

A = randn(x_dim);
B = randn(x_dim,1);
C = randn(1,x_dim);
D = randn(1,1);

for i=1:300
    u_rand = randn(1,1);
    u_traj = [u_traj; u_rand];
    y_traj = [y_traj; C * x + D * u_rand];
    x = A * x + B * u_rand;
end



ddmpc= DDMPC(u_traj,y_traj,Q,R,n,l);
u = 0;
x = randn(x_dim,1);

for i=1:100
    y = C * x + D * u;
    x = A * x + B * u;
    u = ddmpc.step(u,y);
    ddmpc.y_measure;
    [y,u]
end
