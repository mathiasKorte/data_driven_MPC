function bool = check_persistently_exciting(H)
    threshold = 0.01;
    L = size(H,1);
    sigma = svd(H);
    bool = false;
    if(length(sigma)>=L)
        if(sigma(L)>=threshold)
            bool = true;
        end
    end
end