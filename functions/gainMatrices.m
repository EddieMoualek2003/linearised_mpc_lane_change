function [kx, ku] = gainMatrices(mpcStepFunction, x0, u0)
    % Nudge each state, then each previous input, up and down by h.
    % Solves the unconstrained problem u(k) = kx*x(k) + ku*u(k-1)
    % Change each one at a time 
    h = 1e-3;
    kx = zeros(numel(u0), numel(x0));
    for j = 1:numel(x0)
        dx = zeros(size(x0));
        dx(j) = h;
        kx(:, j) = (mpcStepFunction(x0 + dx, u0) - mpcStepFunction(x0 - dx, u0))/(2*h);
    end
    
    ku = zeros(2, 2);
    for j = 1:2
        du = zeros(2, 1);
        du(j) = h;
        ku(:, j) = (mpcStepFunction(x0, u0 + du) - mpcStepFunction(x0, u0 - du)) / (2*h);
    end

end