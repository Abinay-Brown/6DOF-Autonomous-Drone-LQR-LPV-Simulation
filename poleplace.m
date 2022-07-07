function [k1, k2] = poleplace(lambda1, lambda2)
k2 = lambda1 + lambda2;
k1 = (lambda1 - ((lambda1+lambda2)/2))^2 - ((lambda1+lambda2)^2)/4;
end