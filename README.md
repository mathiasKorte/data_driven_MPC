# Data Driven MPC

- Necessary: Trajectory U_d,Y_d, cost-matricies Q and R (need to be positive (semi-)definite and postive values for n and L (steps backwards and forwards)
- Optional: Control variant  "ctrl_mode" $\in$ {"nominal"(default), "robust", "nonlinear"}
- Optional for all MPC variants: reference trajectory $u_{ref}$ or $y_{ref}$ and constrains G_mat_u,G_mat_y,g_vec_u,g_vec_y (default $=0$ for all)
- Optional for Robust/Nonlinear DDMPC: regularization parameter $\lambda_\alpha$, $\lambda_\sigma$ and $\epsilon$ (default $=1$ for all)
