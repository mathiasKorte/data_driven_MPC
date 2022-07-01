function [boundaryMat,boundaryVec] = create_boundryfunction(G_u,G_y,g_u,g_y,L,HL)
    boundaryMat = blkdiag(kron(eye(L),G_u),kron(eye(L),G_y))*HL;
    boundaryVec = [kron(ones(L,1),g_u);kron(ones(L,1),g_y)];
end