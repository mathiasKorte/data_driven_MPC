function bool = check_positiv_semi_definit(A)
    sigma = svd(A);
    bool = (sigma(end)>=0);
end