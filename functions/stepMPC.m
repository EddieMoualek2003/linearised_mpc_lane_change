function u = stepMPC(adaptive, mpcobj, xmpc, s_fun, x, uPrev, r, plantK, nomK)
    xmpc.Plant = x;
    xmpc.LastMove = uPrev; % Artificially inject the previous manipulated variable
    y = s_fun.gF(x, uPrev); % Measure the current output.

    if adaptive
        % Used for the adaptive cases
        u = mpcmoveAdaptive(mpcobj, xmpc, plantK, nomK, y(1:4), r);
    else
        % Used for the LTI case
        u = mpcmove(mpcobj, xmpc, y(1:4), r);
    end
end