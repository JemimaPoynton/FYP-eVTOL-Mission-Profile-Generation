function deriv = meanDeriv(x,y)
% calculates mean derivative

deriv = mean(gradient(y)./gradient(x));