function bool = check_positiv_semi_definit(A)
    eigs = svd(A);
    bool = (eigs(end)>=0);
end