function [mpcobj, xmpc] = buildUnconstrainedMPC(params, s_fun)
    nx = numel(params.xLin);
    nu = numel(params.uLin);

    Ac = s_fun.AcF(params.xLin, params.uLin);
    Bc = s_fun.BcF(params.xLin, params.uLin);
    Cc = s_fun.CcF(params.xLin, params.uLin);
    Dc = s_fun.DcF(params.xLin, params.uLin);
    f0 = s_fun.fF(params.xLin, params.uLin);

    plant_d = c2d(ss(Ac, Bc, Cc, Dc), params.Ts, 'zoh');
    plant_d = setmpcsignals(plant_d, 'MO', 1:4, 'UO', 5:6);

    M    = expm([Ac Bc f0; zeros(nu+1, nx+nu+1)] * params.Ts);
    nom0 = struct('X', params.xLin, 'U', params.uLin, 'Y', s_fun.gF(params.xLin, params.uLin), 'DX', M(1:nx, end));

    mpcobj = mpc(plant_d, params.Ts, params.Np, params.Nc);
    mpcobj.Model.Nominal = nom0;
    mpcobj.Weights.OutputVariables          = [0, 10, 2, 2, 0, 0];   % [X, Y, psi, vx, vy, omega]
    mpcobj.Weights.ManipulatedVariables     = [0, 0];                % [ax, delta]
    mpcobj.Weights.ManipulatedVariablesRate = [2, 100];
    setoutdist(mpcobj, 'model', tf(zeros(6, 1)));
    setEstimator(mpcobj, 'custom');

    xmpc = mpcstate(mpcobj);

end