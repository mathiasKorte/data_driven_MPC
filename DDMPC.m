classdef DDMPC < handle
   properties
        u_d;    % Input trajectory
        y_d;    % Output trajectory
        Q;  % Cost matrix Q
        R;  % Cost matri R
        n;  % Upper bound on system order
        L;  % Prediction horizon
        u_s;    % Setpoint for the input
        y_s;    % Setpoint for the output
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
        G_u;
        G_y;
        g_u;
        g_y;
        boundaryMat;
        boundaryVec;
   end
   methods
       function obj = DDMPC(u_d,y_d,Q,R,n,L,varargin)
          
          pars = inputParser;
          %% Required input variables
          % Validation
          validQ = @(x) (check_positiv_semi_definit(x));
          validR = @(x) (check_positiv_semi_definit(x));
          validn = @(x) (x>0);
          validL = @(x) (x>0);
          
          % Add variables
          addRequired(pars,'u_d');      % Input trajectory
          addRequired(pars,'y_d');      % Output trajectory
          addRequired(pars,'Q',validQ); % Cost matrix Q
          addRequired(pars,'R',validR); % Cost matrix R
          addRequired(pars,'n',validn); % Upperbound on system order
          addRequired(pars,'L',validL); % Prediction horizon
          
          %% Optional input variables
          % Artificial setpoints
          defaultU_s = zeros(1,size(u_d,2));
          defaultY_s = zeros(1,size(y_d,2));
          
          validU_s = @(x) (size(x,1) == 1 && size(x,2) == size(u_d,2));
          validY_s = @(x) (size(x,1) == 1 && size(x,2) == size(y_d,2));
          
          addOptional(pars,'u_s',defaultU_s,validU_s);
          addOptional(pars,'y_s',defaultY_s,validY_s);
          
          % Constraint sets          
          defaultG_u = zeros(0,size(u_d,2));
          defaultG_y = zeros(0,size(y_d,2));
          defaultg_u = zeros(0,1);
          defaultg_y = zeros(0,1);
          
          validG_u = @(x) (size(x,2) == size(u_d,2));
          validG_y = @(x) (size(x,2) == size(y_d,2));
          validg_u = @(x) (size(x,2) == 1);
          validg_y = @(x) (size(x,2) == 1);
          
          addOptional(pars,'G_mat_u',defaultG_u,validG_u);
          addOptional(pars,'G_mat_y',defaultG_y,validG_y);
          addOptional(pars,'g_vec_u',defaultg_u,validg_u);
          addOptional(pars,'g_vec_y',defaultg_y,validg_y);
          
          % TODO: What is the default size of S
          % Design parameter
          defaultS = 1;
          validS = @(x) (x>0);
          addOptional(pars,'S',defaultS,validS); 
          
          % Regularization parameters
          default_lambda_alpha = 1;
          default_lambda_sigma = 1;
          
          valid_lambda_alpha = @(x) (x>0);
          valid_lambda_sigma = @(x) (x>0);
          
          addOptional(pars,'lambda_alpha',default_lambda_alpha,valid_lambda_alpha);
          addOptional(pars,'lambda_alpha',default_lambda_sigma,valid_lambda_sigma);
            
          % Target setpoint
          defaultY_target = zeros(1,size(y_d,2));
          validY_target = @(x) (size(x,1) == 1 && size(x,2) == size(y_d,2));
          addOptional(pars,'y_target',defaultY_target,validY_target);
          
          if any(strcmp(varargin, 'u_s'))
            fprintf('U_S vorhanden');
          else
            fprintf('U_S nicht vorhanden');
          end
       
          parse(pars,u_d,y_d,Q,R,n,L,varargin{:});
          obj.u_d = pars.Results.u_d;
          obj.y_d = pars.Results.y_d;
          obj.Q = pars.Results.Q;
          obj.R = pars.Results.R;
          obj.n = pars.Results.n;
          obj.L = pars.Results.L;
          obj.u_s = pars.Results.u_s;
          obj.y_s = pars.Results.y_s;
          obj.G_u = pars.Results.G_mat_u;
          obj.G_y = pars.Results.G_mat_y;
          obj.g_u = pars.Results.g_vec_u;
          obj.g_y = pars.Results.g_vec_y;
          

          
          nargin
          
          [N_u, obj.m] = size(obj.u_d);
          [N_y, obj.p] = size(obj.y_d);

          % Check Condidition matricies and vectors G_u, G_y, g_u, g_y
          if((size(obj.G_u,1)~=size(obj.g_u,1))||(size(obj.G_y,1)~=size(obj.g_y,1)))
              error("Sizes of Conndtion Matrix G and vector g do not fit")
          end
          
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
          
          QR_diag = blkdiag(kron(eye(L),R),kron(eye(L),Q)); % Create quadratic cost matrix
          obj.costMat = obj.HL'*QR_diag*obj.HL;
          obj.costMat = (obj.costMat+obj.costMat')/2;
          obj.costVec = -2*[kron(ones(L,1),obj.u_s'); kron(ones(L,1),obj.y_s')]'*QR_diag*obj.HL; %zeros(obj.N+1-obj.L-obj.n,1);   % Create cost vector
          

          obj.condMat = obj.Hn; % Create condition matrix for quadprog solver 

          obj.boundaryMat = blkdiag(kron(eye(L),obj.G_u),kron(eye(L),obj.G_y))*obj.HL;
          obj.boundaryVec = [kron(ones(obj.L,1),obj.g_u);kron(ones(obj.L,1),obj.g_y)];
          
          
          obj.u_measure=zeros(obj.n*obj.m,1);
          obj.y_measure=zeros(obj.n*obj.p,1);
       end
       
       function u_next = step(obj,u_measure_new, y_measure_new)

           obj.u_measure = [obj.u_measure; u_measure_new];  % Stacked input measurement trajectories
           obj.y_measure = [obj.y_measure; y_measure_new];  % Stacked output measurement trajectories

           condVec = [obj.u_measure(end-obj.n*obj.m+1:end);obj.y_measure(end-obj.n*obj.m+1:end)];   % Create condition vector for quadprog solver

           options = optimoptions('quadprog','Display','off');%, 'MaxIterations',2.0e+05);   % Define options for quadprog solver
           
           alpha = quadprog(obj.costMat,obj.costVec,obj.boundaryMat,obj.boundaryVec,obj.condMat,condVec, [],[],[],options);

           cost = alpha' * obj.costMat * alpha
           cond = max(abs(obj.condMat * alpha-condVec))

           u_next = obj.HL_u(1:obj.m,:) * alpha;
      end
   end
end