function [T_lti, T_ltv, T_cmp] = analyseRuns(runs_lti, runs_ltv)
% Lane-change tracking metrics for the LTI and LTV controllers.
% Compares each controller across its speed sweep, then LTV against LTI
% at matching (vxDesign, vxSim) points.

T_lti = metricsTable(runs_lti);
T_ltv = metricsTable(runs_ltv);

% Parameter variation within each controller
fprintf('\nLTI runs:\n');
disp(T_lti);
fprintf('\nLTV runs:\n');
disp(T_ltv);

plotFamily(runs_lti, T_lti, 'LTI');
plotFamily(runs_ltv, T_ltv, 'LTV');
plotTrends(T_lti, T_ltv);

% LTV vs LTI at each operating point both controllers were run at
keys = intersect(T_lti.Properties.RowNames, T_ltv.Properties.RowNames, 'stable');
unmatched = setxor(T_lti.Properties.RowNames, T_ltv.Properties.RowNames);
if ~isempty(unmatched)
    warning('No matching run for: %s', strjoin(unmatched, ', '));
end

costLti = toCost(T_lti(keys, :));
costLtv = toCost(T_ltv(keys, :));

% subtract metric columns only, keep the operating point columns as they are
metricCols = setdiff(costLti.Properties.VariableNames, ...
    {'vxDesign', 'vxSim', 'mismatch'}, 'stable');
T_cmp = costLtv;
T_cmp{:, metricCols} = costLtv{:, metricCols} - costLti{:, metricCols};

fprintf('\nLTV - LTI (negative means LTV is better):\n');
disp(T_cmp);

plotCompare(runs_lti, runs_ltv, keys);
end


function T = metricsTable(runs)
keys = string(fieldnames(runs));

% fill backwards so m is allocated at full size on the first pass
for i = numel(keys):-1:1
    m(i) = runMetrics(runs.(keys(i)));
end

T = struct2table(m, 'RowNames', cellstr(keys));
T = sortrows(T, {'vxDesign', 'vxSim'});
end


function m = runMetrics(r)
p = r.params;
t = r.t;
y = r.Y(:, 2);
yRef = r.Ref(:, 2);
e = y - yRef;

laneWidth = yRef(end);
rateLimit = p.Jlat_design * p.Ts * p.m / p.Cf;   % max steering step per sample
nLast = round(2 / p.Ts);                          % samples in the last 2 s

m.vxDesign = p.vxDesign;
m.vxSim = p.vxSim;
m.mismatch = p.vxSim - p.vxDesign;

m.RMSe = sqrt(mean(e.^2));
m.maxErr = max(abs(e));
m.lag95 = firstCross(t, y, 0.95*laneWidth) - firstCross(t, yRef, 0.95*laneWidth);
m.settle2 = settleTime(t, y, laneWidth, 0.02) - settleTime(t, yRef, laneWidth, 0.02);
m.ssErr = mean(e(end-nLast+1:end));
m.over = max(y) - laneWidth;

% % of samples where the steering rate is sitting on its limit
m.atLim = 100 * mean(abs(abs(diff(r.U(:, 2))) - rateLimit) < 1e-6);

m.maxAy = max(abs(r.AY));
m.ayViol = 100 * mean(abs(r.AY) > p.ay_max);
m.maxJerk = max(abs(diff(r.AY))) / p.Ts;
m.maxSlack = max(r.Slack);
m.QPfail = sum(r.QP < 0);
end


function tc = firstCross(t, y, level)
i = find(y >= level, 1);
if isempty(i)
    tc = NaN;
else
    tc = t(i);
end
end


function ts = settleTime(t, y, target, tol)
i = find(abs(y - target) > tol*target, 1, 'last');
if isempty(i)
    ts = t(1);
elseif i == numel(y)
    ts = NaN;   % still outside the band at the end, never settled
else
    ts = t(i+1);
end
end


function T = toCost(T)
% Make every metric "smaller is better" so the LTV - LTI differences
% all read the same way
T.lag95 = abs(T.lag95);
T.settle2 = abs(T.settle2);
T.ssErr = abs(T.ssErr);
T.over = max(T.over, 0);
end


function plotFamily(runs, T, name)
keys = T.Properties.RowNames;
r0 = runs.(keys{1});

figure('Name', [name ' - parameter variation']);
tiledlayout(3, 1, 'TileSpacing', 'compact');

ax1 = nexttile;
hold on
grid on
plot(ax1, r0.t, r0.Ref(:, 2), 'k--', 'DisplayName', 'ref');
ax1.ColorOrderIndex = 1;   % the black ref line still uses up a colour, so reset

ax2 = nexttile;
hold on
grid on

ax3 = nexttile;
hold on
grid on

for i = 1:numel(keys)
    r = runs.(keys{i});
    plot(ax1, r.t, r.Y(:, 2), 'DisplayName', keys{i});
    plot(ax2, r.t, r.Y(:, 2) - r.Ref(:, 2), 'DisplayName', keys{i});
    plot(ax3, r.t, r.AY, 'DisplayName', keys{i});
end

yline(ax3, [-1 1] * r0.params.ay_max, 'k:', 'HandleVisibility', 'off');

title(ax1, [name ': parameter variation']);
ylabel(ax1, 'Y (m)');
ylabel(ax2, 'e_Y (m)');
ylabel(ax3, 'a_y (m/s^2)');
xlabel(ax3, 't (s)');

legend(ax1, 'Interpreter', 'none', 'Location', 'southeast');
linkaxes([ax1 ax2 ax3], 'x');
end


function plotTrends(T_lti, T_ltv)
metrics = {'RMSe', 'maxErr', 'lag95', 'maxAy'};
labels = {'RMS e_Y (m)', 'max |e_Y| (m)', 'lag_{95} (s)', 'max |a_y| (m/s^2)'};

designSpeeds = unique([T_lti.vxDesign; T_ltv.vxDesign]);
colours = lines(numel(designSpeeds));

figure('Name', 'Metric trends vs vxSim');
tiledlayout(2, 2, 'TileSpacing', 'compact');

for k = 1:numel(metrics)
    nexttile
    hold on
    grid on

    plotTrendLines(T_lti, metrics{k}, designSpeeds, colours, '-o', 'LTI');
    plotTrendLines(T_ltv, metrics{k}, designSpeeds, colours, '--s', 'LTV');

    xlabel('v_{x,sim} (m/s)');
    ylabel(labels{k});
end

legend('Location', 'bestoutside');
end


function plotTrendLines(T, metric, designSpeeds, colours, style, name)
for d = 1:numel(designSpeeds)
    rows = T.vxDesign == designSpeeds(d);
    if ~any(rows)
        continue
    end
    plot(T.vxSim(rows), T.(metric)(rows), style, 'Color', colours(d, :), ...
        'DisplayName', sprintf('%s, v_{design}=%g', name, designSpeeds(d)));
end
end


function plotCompare(runs_lti, runs_ltv, keys)
figure('Name', 'LTI vs LTV');
tiledlayout('flow', 'TileSpacing', 'compact');

for i = 1:numel(keys)
    a = runs_lti.(keys{i});
    b = runs_ltv.(keys{i});

    nexttile
    hold on
    grid on
    plot(a.t, a.Y(:, 2) - a.Ref(:, 2), 'DisplayName', 'LTI');
    plot(b.t, b.Y(:, 2) - b.Ref(:, 2), 'DisplayName', 'LTV');

    title(keys{i}, 'Interpreter', 'none');
    ylabel('e_Y (m)');
    xlabel('t (s)');
    legend('Location', 'best');
end
end