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
        p;  % Output dimension
        HLn_u;  % Input Hankel matrix: H_(L+n)(u^d)
        HLn_y;  % Output Hankel matrix: H_(L+n)(d^d)
        costMat;    % Quadratic cost matrix for quadprog solver
        costVec;    % Cost vector for quadprog solver
        condMat;    % Condition matrix for quadprog solver
        Hn_u;   
        Hn_y;
        HL_u;
        HL_y;
        HL;    % Hankel matrix [H_L(u^d); H_L(y^d)]
        Hn;    % Hankel matrix [H_n(u^d); H_n(y^d)]
        u_measure;
        y_measure;
   end
   methods
       function obj = DDMPC(u_d,y_d,Q,R,n,L)
          
          pars = inputParser;
          validQ = @(x) (check_positiv_semi_definit(x));
          validR = @(x) (check_positiv_semi_definit(x));
          validn = @(x) (x>0);
          validL = @(x) (x>0);
          
          addRequired(pars,'u_d');
          addRequired(pars,'y_d');
          addRequired(pars,'Q',validQ);
          addRequired(pars,'R',validR);
          addRequired(pars,'n',validn);
          addRequired(pars,'L',validL);
          
          parse(pars,u_d,y_d,Q,R,n,L);
          obj.u_d = pars.Results.u_d;
          obj.y_d = pars.Results.y_d;
          obj.Q = pars.Results.Q;
          obj.R = pars.Results.R;
          obj.n = pars.Results.n;
          obj.L = pars.Results.L;
          
          [N_u, obj.m] = size(obj.u_d);
          [N_y, obj.p] = size(obj.y_d);
          
          % Check size of cost matrices
          if ~isequal(size(obj.Q), [obj.p obj.p])
               error("Size of costmatrix Q is not correct")
          end
          if ~isequal(size(obj.R), [obj.m obj.m])
               error("Size of costmatrix R is not correct")
          end
          
          % Check that input and output length are the same
          if ~(N_u == N_y)
              error("Input and output trajectory length don't fit")
          end
          
          obj.N = N_u;
          
          % Check required data length
          if ~(obj.N >= (obj.m+1)*obj.L-1)
              error("Lower bound on required data length N not full filled")
          end
          
          % Create input Hankel matrix: H_(L+n)(u^d)
          obj.HLn_u = create_Hankel(obj.u_d, obj.L, obj.n);
          % Create output Hankel matrix: H_(L+n)(y^d)
          obj.HLn_y = create_Hankel(obj.y_d, obj.L, obj.n);

          
          % Check if input data is persistntly exciting of order L
          if ~(check_persistently_exciting(obj.HLn_u))
              error("Input sequence is not persistently exciting of order " + (obj.L+obj.n))
          end
          
          obj.Hn_u = obj.HLn_u(1:obj.n*obj.m,:);
          obj.Hn_y = obj.HLn_y(1:obj.n*obj.p,:);
          obj.HL_u = obj.HLn_u(obj.n*obj.m+1:end,:);
          obj.HL_y = obj.HLn_y(obj.n*obj.p+1:end,:);
          
          obj.Hn = [obj.Hn_u; obj.Hn_y];    % Create combined Hankel matrix history
          obj.HL = [obj.HL_u; obj.HL_y];    % Create combined Hankel matrix future
          
          obj.costMat = blkdiag(kron(eye(L),Q),kron(eye(L),R)); % Create quadratic cost matrix
          obj.costMat = obj.HL'*obj.costMat*obj.HL;
          obj.costMat = (obj.costMat+obj.costMat')/2;
          obj.costVec = ones(obj.N+1-obj.L-obj.n,1)/100;   % Create cost vector
          
          obj.condMat = obj.Hn; % Create condition matrix for quadprog solver (can be further enhanced)
          obj.u_measure=zeros(obj.n*obj.m,1);
          obj.y_measure=zeros(obj.n*obj.p,1);
       end
       
       function u_next = step(obj,u_measure_new, y_measure_new)

           obj.u_measure = [obj.u_measure; u_measure_new];  % Stacked input measurement trajectories
           obj.y_measure = [obj.y_measure; y_measure_new];  % Stacked output measurement trajectories

           condVec = [obj.u_measure(end-obj.n*obj.m+1:end);obj.y_measure(end-obj.n*obj.m+1:end)];   % Create condition vector for quadprog solver
       
           options = optimoptions('quadprog','Display','off');%, 'MaxIterations',2.0e+05);   % Define options for quadprog solver
           
           alpha = quadprog(obj.costMat,obj.costVec,[],[],obj.condMat,condVec, [],[],[],options);

%            cost = alpha' * obj.costMat * alpha
%            cond = max(abs(obj.condMat * alpha-condVec))

           u_next = obj.HL_u(1:obj.m,:) * alpha;
      end
   end
end