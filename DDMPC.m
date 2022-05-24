classdef DDMPC < handle
   properties
      H
      Aeq
      alpha_dim
      u_measure
      y_measure
      U_d_next
      n
      test = 1;
   end
   methods
       function obj = DDMPC(u_traj,y_traj,Q,R,n,l)
          obj.alpha_dim = length(u_traj)+1-l-n;
          U_d = hankel(u_traj);
          U_d = U_d(1:n+l, 1:obj.alpha_dim);
          Y_d = hankel(y_traj);
          Y_d = Y_d(1:n+l, 1:length(y_traj)+1-l-n); 
          
          obj.Aeq = [U_d(1:n,1:end); Y_d(1:n,1:end)];

          a = [U_d(n+1:end,1:end); Y_d(n+1:end,1:end)];
          b =  blkdiag(kron(eye(l),Q),kron(eye(l),R));
          obj.H = a' * b * a;
          obj.H = (obj.H+obj.H')/2;

          obj.u_measure=zeros(n,1);
          obj.y_measure=zeros(n,1);

          obj.U_d_next = U_d(n+1, 1:end);
          obj.n = n;
       end
       function u_next = step(obj,u_measure_new, y_measure_new)

           obj.u_measure = [obj.u_measure; u_measure_new];
           obj.y_measure = [obj.y_measure; y_measure_new];

           beq = [obj.u_measure(end-obj.n+1:end);obj.y_measure(end-obj.n+1:end)];

           f = zeros(obj.alpha_dim,1);           
           options = optimoptions('quadprog','Display','off');
           
           alpha = quadprog(obj.H,f,[],[],obj.Aeq,beq, [],[],[],options);

           u_next = obj.U_d_next * alpha;
      end
   end
end