classdef DDMPC < handle
   properties
%       H
%       Aeq
%       alpha_dim
%       u_measure
%       y_measure
%       U_d_next
%       n
%       test = 1;
        u_d;    % Input trajectory
        y_d;    % Output trajectory
        Q;  % Cost matrix Q
        R;  % Cost matri R
        n;  % Upper bound on system order
        L;  % Prediction horizon
        N;  % Input trajectory length
        m;  % Input dimension
   end
   methods
       function obj = DDMPC(u_d,y_d,Q,R,n,L)
          
          p = inputParser;
          validQ = @(x) (x>0);
          validR = @(x) (x>0);
          validn = @(x) (x>0);
          validL = @(x) (x>0);
          
          addRequired(p,'u_d');
          addRequired(p,'y_d');
          addRequired(p,'Q',validQ);
          addRequired(p,'R',validR);
          addRequired(p,'n',validn);
          addRequired(p,'L',validL);
          
          parse(p,u_d,y_d,Q,R,n,L);
          obj.u_d = p.Results.u_d;
          obj.y_d = p.Results.y_d;
          obj.Q = p.Results.Q;
          obj.R = p.Results.R;
          obj.n = p.Results.n;
          obj.L = p.Results.L;
          
          [obj.N, obj.m] = size(obj.u_d)
%           obj.alpha_dim = length(u_traj)+1-L-n;
%           U_d = hankel(u_traj);
%           U_d = U_d(1:n+L, 1:obj.alpha_dim);
%           Y_d = hankel(y_traj);
%           Y_d = Y_d(1:n+L, 1:length(y_traj)+1-L-n); 
%           
%           obj.Aeq = [U_d(1:n,1:end); Y_d(1:n,1:end)];
% 
%           a = [U_d(n+1:end,1:end); Y_d(n+1:end,1:end)];
%           b =  blkdiag(kron(eye(L),Q),kron(eye(L),R));
%           obj.H = a' * b * a;
%           obj.H = (obj.H+obj.H')/2;
% 
%           obj.u_measure=zeros(n,1);
%           obj.y_measure=zeros(n,1);
% 
%           obj.U_d_next = U_d(n+1, 1:end);
%           obj.n = n;
       end
       function print(obj)
           obj.u_d
       end
%        function u_next = step(obj,u_measure_new, y_measure_new)
% 
%            obj.u_measure = [obj.u_measure; u_measure_new];
%            obj.y_measure = [obj.y_measure; y_measure_new];
% 
%            beq = [obj.u_measure(end-obj.n+1:end);obj.y_measure(end-obj.n+1:end)];
% 
%            f = zeros(obj.alpha_dim,1);           
%            options = optimoptions('quadprog','Display','off');
%            
%            alpha = quadprog(obj.H,f,[],[],obj.Aeq,beq, [],[],[],options);
% 
%            u_next = obj.U_d_next * alpha;
%       end
   end
end