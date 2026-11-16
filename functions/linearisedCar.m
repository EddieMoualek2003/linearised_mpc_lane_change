function [A, B, plantK, nomK] = linearisedCar(s_fun, x, u, Ts)
% Same discretisation as ltvControllerNLPlant: A, B for the car, and the
% plant and nominal structs that mpcmoveAdaptive expects
nx = numel(x);
nu = numel(u);
Ac = s_fun.AcF(x, u);
Bc = s_fun.BcF(x, u);
f0 = s_fun.fF(x, u);
M  = expm([Ac Bc f0; zeros(nu+1, nx+nu+1)] * Ts);
A  = M(1:nx, 1:nx);
B  = M(1:nx, nx+1:nx+nu);
plantK = ss(A, B, s_fun.CcF(x, u), s_fun.DcF(x, u), Ts);
nomK   = struct('X', x, 'U', u, 'Y', s_fun.gF(x, u), 'DX', M(1:nx, end));
end