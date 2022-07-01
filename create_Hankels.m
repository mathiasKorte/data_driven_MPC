function [Hn, HL] = create_Hankels(u_d,y_d, L, n,m,p)
  % Create input Hankel matrix: H_(L+n)(u^d)
  HLn_u = create_Hankel(u_d, L, n);
  % Create output Hankel matrix: H_(L+n)(y^d)
  HLn_y = create_Hankel(y_d, L, n);

  
  % Check if input data is persistntly exciting of order L
%   if ~(check_persistently_exciting(HLn_u))
%       error("Input sequence is not persistently exciting of order " + (L+n))
%   end
  
  Hn_u = HLn_u(1:n*m,:);
  Hn_y = HLn_y(1:n*p,:);
  HL_u = HLn_u(n*m+1:end,:);
  HL_y = HLn_y(n*p+1:end,:);
  
  Hn = [Hn_u; Hn_y];    % Create combined Hankel matrix history
  HL = [HL_u; HL_y];    % Create combined Hankel matrix future
end