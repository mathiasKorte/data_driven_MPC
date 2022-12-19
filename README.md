# Data Driven MPC
This project was created by Stephan Rappensberger and Mathias Korte

## DDMPC.m - Class

### Constructor
The constructer generates the DDMPC-class with user given paramters. These parameters can/must specified:

#### Necessary:
- matrcies U_d $\in\mathcal{R}^{N\times m}$ and Y_d $\in\mathcal{R}^{N\times p}$ contain the recorded input-output trajectory of length $N$ of a sythem with $u\in\mathcal{R}^{m}$ and $y\in\mathcal{R}^{p}$
- positiv definite cost-matricies Q$\in\mathcal{R}^{m\times m}$ and R$\in\mathcal{R}^{p\times p}$ 
- postive values for n and L (n is the number of past $u_k$/$y_k$ considered for including the state information and L is number of steps the opimization problems solves into the future. It follows that complexer systems can only be solved with bigger n and L.)

#### Optional
- Control variant "ctrl_mode" $\in$ {"nominal"(default), "robust", "nonlinear"}
- Reference trajectory u_ref$\in\mathcal{R}^{m}$ and y_ref$\in\mathcal{R}^{p}$ (by default both are set to 0)
- Any number of input/output constrains: For $d$ input and $f$ output constrains: G_mat_u$\in\mathcal{R}^{d\times m}$, g_vec_u$\in\mathcal{R}^{d}$, G_mat_y$\in\mathcal{R}^{f\times p}$ and g_vec_y$\in\mathcal{R}^{f}$  (emty matrcies by default with $d=f=0$)
- Regularization parameters for Robust/Nonlinear DDMPC:  $\lambda_\alpha$, $\lambda_\sigma$ and $\epsilon$ (default $=1$ for all)

### Step function
- function inputs: new measuremnts u_m$\in\mathcal{R}^{m}$ and y_m$\in\mathcal{R}^{p}$
- returns: control input  u$\in\mathcal{R}^{m}$ 

## Evalutaion 
For purposes of evaluation we included two files for testing our DDMPC:
- dc_motor_example.m: for the linear sysems with state space representation
- four_tank_example.m: for a nonlinear system
