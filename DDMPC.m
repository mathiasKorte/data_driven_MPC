classdef DDMPC < handle
   properties
       u_d,y_d,Q,R,n,L,u_s,y_s,G_u,G_y,g_u,g_y,m,p,N,alpha_dim,sigma_dim,Hn,HL,costMat,costVec,condMat,boundaryMat,boundaryVec,u_measure,y_measure,lambda_alpha,lambda_sigma,epsilon,ctrl_mode
   end
   methods
       function obj = DDMPC(u_d,y_d,Q,R,n,L,varargin)
          
          pars = inputParser;
          
          [obj.u_d,obj.y_d,obj.Q,obj.R,obj.n,obj.L,obj.u_s, ....
              obj.y_s,obj.G_u,obj.G_y,obj.g_u,obj.g_y,obj.m,....
              obj.p,obj.N,obj.alpha_dim,obj.sigma_dim, ....
              obj.lambda_alpha,obj.lambda_sigma,obj.epsilon, ....
              obj.ctrl_mode] = load_inputs(pars,u_d,y_d,Q,R,n,L,varargin);
          
          [obj.Hn, obj.HL] = create_Hankels(obj.u_d,obj.y_d,obj.L,obj.n,obj.m,obj.p,obj.Hn,obj.HL,obj.ctrl_mode);
          
          [obj.costMat, obj.costVec] = create_costfunction(obj.Q,obj.R,obj.L,obj.HL,obj.u_s,obj.y_s);
          
          obj.condMat = obj.Hn;  

          [obj.boundaryMat,obj.boundaryVec] = create_boundryfunction(obj.G_u,obj.G_y,obj.g_u,obj.g_y,obj.L,obj.HL);
         
          if(strcmp(obj.ctrl_mode,"robust"))          
              [obj.costMat,obj.costVec,obj.condMat,obj.boundaryMat,obj.boundaryVec] = add_robustness(obj.costMat,obj.costVec,obj.condMat,obj.boundaryMat,obj.boundaryVec,obj.lambda_alpha,obj.lambda_sigma,obj.alpha_dim,obj.sigma_dim,obj.epsilon);
          end
          
          obj.u_measure=zeros(obj.n*obj.m,1);
          obj.y_measure=zeros(obj.n*obj.p,1);
       end
       
       function u_next = step(obj,u_measure_new, y_measure_new)
           
           obj.u_measure = [obj.u_measure; u_measure_new];  % Stacked input measurement trajectories
           obj.y_measure = [obj.y_measure; y_measure_new];  % Stacked output measurement trajectories

           condVec = [obj.u_measure(end-obj.n*obj.m+1:end);obj.y_measure(end-obj.n*obj.p+1:end)];   % Create condition vector for quadprog solver

           if(strcmp(obj.ctrl_mode,"nonlinear"))
               obj.u_d = [obj.u_d(2:end,:); u_measure_new'];
               obj.y_d = [obj.y_d(2:end,:); y_measure_new'];
                
               [obj.Hn, obj.HL] = create_Hankels(obj.u_d,obj.y_d,obj.L,obj.n,obj.m,obj.p,obj.Hn,obj.HL,obj.ctrl_mode);
               [obj.costMat, obj.costVec] = create_costfunction(obj.Q,obj.R,obj.L,obj.HL,obj.u_s,obj.y_s);
               obj.condMat = obj.Hn;
               [obj.boundaryMat,obj.boundaryVec] = create_boundryfunction(obj.G_u,obj.G_y,obj.g_u,obj.g_y,obj.L,obj.HL);
               [obj.costMat,obj.costVec,obj.condMat,obj.boundaryMat,obj.boundaryVec] = add_robustness(obj.costMat,obj.costVec,obj.condMat,obj.boundaryMat,obj.boundaryVec,obj.lambda_alpha,obj.lambda_sigma,obj.alpha_dim,obj.sigma_dim,obj.epsilon);
               obj.condMat = [obj.condMat;[ones(1,obj.alpha_dim),zeros(1,obj.sigma_dim)]];
               condVec = [condVec;1];
           end
           
           alpha_sigma = quadprog(obj.costMat,obj.costVec,obj.boundaryMat,obj.boundaryVec,obj.condMat,condVec, [],[],[]);

           u_next = obj.HL(1:obj.m,:) * alpha_sigma(1:obj.alpha_dim);
      end
   end
end