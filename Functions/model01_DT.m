function xk1 = model01_DT(xk, uk, Ts)
% Dicretization with Forward Euler Method.
% The discretized function is the continuos model "model01"

M = 1;                         % simulation time
delta = Ts/M;                   % discretizaion step
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*model01(xk1,uk);
end