function [costMat,costVec,condMat,boundaryMat,boundaryVec] = add_robustness(costMat,costVec,condMat,boundaryMat,boundaryVec,lambda_alpha,lambda_sigma,alpha_dim,sigma_dim,epsilon)
    costMat = blkdiag(costMat+lambda_alpha*epsilon*eye(alpha_dim),lambda_sigma/epsilon*eye(sigma_dim));
    costVec = [costVec, zeros(1,sigma_dim)];
    condMat = [condMat,[zeros(size(condMat,1)-sigma_dim,sigma_dim);-eye(sigma_dim)]];
    boundaryMat = blkdiag(boundaryMat,zeros(sigma_dim));
    boundaryVec = [boundaryVec;zeros(sigma_dim,1)];
end