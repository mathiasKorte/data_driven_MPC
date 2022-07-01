function [costMat, costVec] = create_costfunction(Q,R,L,HL,u_s,y_s)
  QR_diag = blkdiag(kron(eye(L),R),kron(eye(L),Q)); % Create quadratic cost matrix
  costMat = HL'*QR_diag*HL;
  costMat = (costMat+costMat')/2;
  costVec = -1*[kron(ones(L,1),u_s'); kron(ones(L,1),y_s')]'*QR_diag*HL; %zeros(N+1-L-n,1);   % Create cost vector
end