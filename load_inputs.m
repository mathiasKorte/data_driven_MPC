function [u_d,y_d,Q,R,n,L,u_s,y_s,G_u,G_y,g_u,g_y,m,p,N,alpha_dim,sigma_dim] = load_inputs(pars,u_d,y_d,Q,R,n,L,vgr)


defaultY_s = zeros(1,size(y_d,2));
defaultG_u = zeros(0,size(u_d,2));
defaultG_y = zeros(0,size(y_d,2));
defaultg_u = zeros(0,1);
defaultg_y = zeros(0,1);
defaultU_s = zeros(1,size(u_d,2));

validQ = @(x) (check_positiv_semi_definit(x));
validR = @(x) (check_positiv_semi_definit(x));
validn = @(x) (x>0);
validL = @(x) (x>0);
validU_s = @(x) (size(x,1) == 1 && size(x,2) == size(u_d,2));
validY_s = @(x) (size(x,1) == 1 && size(x,2) == size(y_d,2));
validG_u = @(x) (size(x,2) == size(u_d,2));
validG_y = @(x) (size(x,2) == size(y_d,2));
validg_u = @(x) (size(x,2) == 1);
validg_y = @(x) (size(x,2) == 1);

addRequired(pars,'u_d');
addRequired(pars,'y_d');
addRequired(pars,'Q',validQ);
addRequired(pars,'R',validR);
addRequired(pars,'n',validn);
addRequired(pars,'L',validL);
addOptional(pars,'u_s',defaultU_s,validU_s);
addOptional(pars,'y_s',defaultY_s,validY_s);
addOptional(pars,'G_mat_u',defaultG_u,validG_u);
addOptional(pars,'G_mat_y',defaultG_y,validG_y);
addOptional(pars,'g_vec_u',defaultg_u,validg_u);
addOptional(pars,'g_vec_y',defaultg_y,validg_y);


parse(pars,u_d,y_d,Q,R,n,L,vgr{:});
u_d = pars.Results.u_d;
y_d = pars.Results.y_d;
Q = pars.Results.Q;
R = pars.Results.R;
n = pars.Results.n;
L = pars.Results.L;
u_s = pars.Results.u_s;
y_s = pars.Results.y_s;
G_u = pars.Results.G_mat_u;
G_y = pars.Results.G_mat_y;
g_u = pars.Results.g_vec_u;
g_y = pars.Results.g_vec_y;

[N_u, m] = size(u_d);
[N_y, p] = size(y_d);


% Check Condidition matricies and vectors G_u, G_y, g_u, g_y
if((size(G_u,1)~=size(g_u,1))||(size(G_y,1)~=size(g_y,1)))
  error("Sizes of Conndtion Matrix G and vector g do not fit")
end

% Check size of cost matrices
if ~isequal(size(Q), [p p])
   error("Size of costmatrix Q is not correct")
end
if ~isequal(size(R), [m m])
   error("Size of costmatrix R is not correct")
end

% Check that input and output length are the same
if ~(N_u == N_y)
  error("Input and output trajectory length don't fit")
end

N = N_u;
alpha_dim = N+1-L-n;
sigma_dim = p*n;

% Check required data length
if ~(N >= (m+1)*L-1)
  error("Lower bound on required data length N not full filled")
end
end