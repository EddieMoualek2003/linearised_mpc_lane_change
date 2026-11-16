function [T, T_cmp] = analyseNMPC(files, results)
% analyseNMPC  Tracking, constraint, solver and timing metrics for NMPC runs.
%
%   T = analyseNMPC(files)
%       files: list of NMPC .mat files, e.g.
%       ["data/nl_controller_data_vx20.mat", "data/nl_controller_data_vx5.mat"]
%       Prints a short report per run and returns one table row per run.
%
%   [T, T_cmp] = analyseNMPC(files, results)
%       Also lines each NMPC run up against LTV, LTI-15 and LTI-25 at the
%       same vxSim, using results.vxDes15 / results.vxDes25 (T_lti, T_ltv)
%       from robustLaneChangeMain.
%
% Tracking metrics use the same definitions as analyseRuns (runMetrics):
% lateral error e = p_y - p_y,ref over the whole run, lag95 and settle2
% relative to the reference's own crossing/settling, 2 s steady-state window.

files = string(files);
names = strings(1, numel(files));
for i = numel(files):-1:1
    S = load(files(i), "xLog", "uLog", "ayLog", "ref", "params", "tSolve", "exitFlag");
    need = ["xLog", "uLog", "ayLog", "ref", "params", "tSolve", "exitFlag"];
    miss = need(~isfield(S, need));
    assert(isempty(miss), "%s is missing: %s", files(i), strjoin(miss, ", "));

    m(i) = runMetrics(S);
    [~, names(i)] = fileparts(files(i));
end

for i = 1:numel(files)
    printReport(names(i), m(i));
end

T = struct2table(m, "RowNames", cellstr(names));
T = sortrows(T, "vxSim");
fprintf("\n=== NMPC summary ===\n");
disp(T);

T_cmp = [];
if nargin > 1
    T_cmp = compareAtSpeed(T, results);
    fprintf("\n=== NMPC vs linearised controllers at the same vxSim ===\n");
    disp(T_cmp);
end
end

%% ---------- Metrics ----------
function m = runMetrics(S)
    p  = S.params;
    n  = size(S.xLog, 1);
    t  = (0:n-1)' * p.Ts;              % row k = state at t = (k-1)*Ts
    y  = S.xLog(:, 2);                 % lateral position p_y
    yr = S.ref(1:n, 2);                % reference p_y on the same rows
    w  = yr(end);                      % lane offset, taken from the reference
    e  = y - yr;
    nSS = round(2 / p.Ts);             % last 2 s for steady state
    dd  = p.mvRate(2);                 % steering-rate limit per sample
    ts  = S.tSolve(2:end);             % first call excluded, as for LTI/LTV

    m.vxSim     = p.vxSim;
    m.RMSe      = sqrt(mean(e.^2));
    m.maxErr    = max(abs(e));
    m.lag95     = firstCross(t, y, 0.95*w) - firstCross(t, yr, 0.95*w);
    m.settle2   = settleTime(t, y, w, 0.02) - settleTime(t, yr, w, 0.02);
    m.ssErr     = mean(e(end-nSS+1:end));
    m.over      = max(y) - w;
    % fmincon meets the rate limit only to its tolerance, so "at the limit"
    % means within 0.1% of it (analyseRuns uses an absolute 1e-6 for the QP)
    m.atLim     = 100 * mean(abs(diff(S.uLog(:, 2))) >= (1 - 1e-3) * dd);
    m.maxAy     = max(abs(S.ayLog));
    m.ayViol    = 100 * mean(abs(S.ayLog) > p.ay_max);
    m.maxJerk   = max(abs(diff(S.ayLog))) / p.Ts;
    m.nConv     = sum(S.exitFlag > 0);
    m.nIterLim  = sum(S.exitFlag == 0);
    m.nFail     = sum(S.exitFlag < 0);
    m.tMed      = median(ts);          % seconds
    m.tP99      = prctile(ts, 99);
    m.tMax      = max(ts);
    m.xRealTime = m.tMed / p.Ts;       % median step time / sample time
end

function tc = firstCross(t, y, lvl)
    i = find(y >= lvl, 1);
    if isempty(i), tc = NaN; else, tc = t(i); end
end

function ts = settleTime(t, y, w, tol)
    i = find(abs(y - w) > tol*w, 1, 'last');
    if isempty(i),        ts = t(1);
    elseif i == numel(y), ts = NaN;    % never settles
    else,                 ts = t(i+1);
    end
end

%% ---------- Report ----------
function printReport(name, m)
    fprintf("\n=== %s (vxSim = %g m/s) ===\n", name, m.vxSim);
    fprintf("Tracking : RMS e_y %.3f m | max |e_y| %.3f m | lag95 %.2f s | settle2 %.2f s | overshoot %.3f m\n", ...
        m.RMSe, m.maxErr, m.lag95, m.settle2, m.over);
    fprintf("a_y      : peak %.3f m/s^2 | above limit %.1f%% of samples | max jerk %.2f m/s^3\n", ...
        m.maxAy, m.ayViol, m.maxJerk);
    fprintf("Steering : rate at limit on %.1f%% of steps\n", m.atLim);
    fprintf("Solver   : %d converged | %d at iteration limit | %d failed\n", ...
        m.nConv, m.nIterLim, m.nFail);
    fprintf("Timing   : median %.2f s | p99 %.2f s | max %.2f s per step (%.0fx the sample time)\n", ...
        m.tMed, m.tP99, m.tMax, m.xRealTime);
end

%% ---------- Comparison with LTI / LTV ----------
function T_cmp = compareAtSpeed(T, results)
    mets = ["RMSe", "maxErr", "lag95", "settle2", "maxAy", "ayViol"];
    des  = string(fieldnames(results));
    des  = des(startsWith(des, "vxDes"));      % vxDes15, vxDes25
    rowNames = {};
    vals     = {};
    for i = 1:height(T)
        v = T.vxSim(i);
        rowNames{end+1, 1} = sprintf('NMPC @ %g m/s', v);
        vals{end+1, 1}     = T{i, mets};
        % LTV is identical at both design speeds, so take it once
        rowNames{end+1, 1} = sprintf('LTV @ %g m/s', v);
        vals{end+1, 1}     = pick(results.(des(1)).T_ltv, v, mets);
        for k = 1:numel(des)
            rowNames{end+1, 1} = sprintf('LTI-%s @ %g m/s', extractAfter(des(k), "vxDes"), v);
            vals{end+1, 1}     = pick(results.(des(k)).T_lti, v, mets);
        end
    end
    T_cmp = array2table(vertcat(vals{:}), "VariableNames", mets, "RowNames", rowNames);
end

function r = pick(Tc, v, mets)
    k = find(abs(Tc.vxSim - v) < 1e-9, 1);
    if isempty(k), r = NaN(1, numel(mets));    % speed not in the sweep
    else,          r = Tc{k, mets};
    end
end