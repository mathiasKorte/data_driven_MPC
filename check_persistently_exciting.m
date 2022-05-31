function bool = check_persistently_exciting(H, m)
    threshold = 0.01;
    L = size(a,1);
    sigma = svd(H);
    bool = false;
    if(length(sigma)>=m*L)
        if(sigma(m*L)>=threshold)
            bool = true;
        end
    end
end