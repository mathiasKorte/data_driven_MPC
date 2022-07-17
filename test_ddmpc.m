close all;
clear all;
clc;

noise = 0;
x_dim = 2;
y_dim = 1;
u_dim = 1;
% A = randn(x_dim);
% [U,S,V]=svd(A);
% A = U*eye(x_dim)*1.1*V';
% B = randn(x_dim,u_dim);
% C = randn(y_dim,x_dim);
% D = randn(y_dim,u_dim);
A=[-1.2,0.03;0.075,-1];
B=[2;0];
C=[0,1];
D=0;

u_d = [];
y_d = [];
x = randn(x_dim,1);
for i=1:50
    u = randn(u_dim,1);
    y = C * x + D * u + randn(y_dim,1)*noise;
    x = A * x + B * u;
    u_d = [u_d; u'];
    y_d = [y_d; y'];
end

figure
subplot(2,1,1)
plot(y_d)
grid on;
title('Y');
subplot(2,1,2)
plot(u_d)
grid on;
title('U')


n = x_dim;     
L = 10;     
R = 0;%eye(u_dim)/1000;    
Q = eye(y_dim);      

ddmpc= DDMPC(u_d,y_d,Q,R,n,L,'y_s',0,....
    'G_mat_u',[1;-1],'g_vec_u',[1;1]*5,....
    ...%'G_mat_y',[1,0;-1,0;0,1;0,-1],'g_vec_y',ones(4,1)*30,....
    'lambda_sigma',100, 'ctrl_mode', 'nominal');
    
u = randn(u_dim,1)*0;
x = randn(x_dim,1)*10;

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

figure
subplot(2,1,1)
plot(y_traj)
grid on;
title('Y');
subplot(2,1,2)
plot(u_traj)
grid on;
title('U')