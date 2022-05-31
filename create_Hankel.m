function H = create_Hankel(tranj, L, n)
    [N,m] = size(tranj);
    H_temp = hankel(1:N);
    H_temp = H_temp(1:L+n, 1:(N-L-n+1));
    H =reshape(tranj(H_temp,:)', (L+n)*m, []);
end