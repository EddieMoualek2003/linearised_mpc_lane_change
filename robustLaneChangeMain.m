%% System
if ~exist('firstRun', 'var')
    addpath("controller_models\", "functions\", "plant_models\");
    firstRun = true;
    mpcverbosity off;
end

%% Run
vxDesignArray = [15 25];
vxSimArray    = linspace(5, 50, 10);

if firstRun
    results = struct();
    for d = vxDesignArray
        res = struct();
        res.vxDesign   = d;
        res.vxSimArray = vxSimArray;
        res.lti_time   = zeros(size(vxSimArray));
        res.ltv_time   = zeros(size(vxSimArray));
        res.runs_lti   = struct();
        res.runs_ltv   = struct();

        for i = 1:numel(vxSimArray)
            params = paramDefs("vxSim", vxSimArray(i), "vxDesign", d);
            fprintf("Test:\t vxDesign: %g \n\t vxSim: %g \n", params.vxDesign, params.vxSim);

            % LTI MPC
            [~, ~, ~, ~, ~, r] = ltiControllerNLPlant(params);
            res.runs_lti.(r.key) = r;

            % Adaptive (LTV) MPC
            [~, ~, ~, ~, ~, r] = ltvControllerNLPlant(params);
            res.runs_ltv.(r.key) = r;
        end
        results.(sprintf("vxDes%d", d)) = res;
    end
    firstRun = false;
else
    fprintf("Data already collected - running analysis\n");
end


%% Data Analysis
outDir = fullfile(fileparts(mfilename("fullpath")), "Figures and Results");
if ~isfolder(outDir), mkdir(outDir); end

for d = vxDesignArray
    f = sprintf("vxDes%d", d);
    [results.(f).T_lti, results.(f).T_ltv, results.(f).T_cmp] = ...
        analyseRuns(results.(f).runs_lti, results.(f).runs_ltv);
end

%% Controller step time vs vxSim
statNames = ["Median", "99th pct."];
cols = lines(numel(vxDesignArray));
fh = figure("Name", "Step time vs vxSim");
tl = tiledlayout(fh, 2, 1, "TileSpacing", "compact");
for s = 1:2
    ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
    for k = 1:numel(vxDesignArray)
        d = vxDesignArray(k);  f = sprintf("vxDes%d", d);
        Sl = stepTimeStats(results.(f).runs_lti);
        Sv = stepTimeStats(results.(f).runs_ltv);
        plot(ax, vxSimArray, Sl(:, s), "-o",  "Color", cols(k,:), ...
            "DisplayName", sprintf("LTI, v_{design}=%d", d));
        plot(ax, vxSimArray, Sv(:, s), "--s", "Color", cols(k,:), ...
            "DisplayName", sprintf("LTV, v_{design}=%d", d));
    end
    ylabel(ax, statNames(s) + " step time (ms)");
end
xlabel(ax, "v_{x,sim} (m/s)");
lg = legend(ax, "Orientation", "horizontal", "NumColumns", 2);
lg.Layout.Tile = "north";

fh.Units = "inches";
fh.Position = [1 1 3.5 3.8];
set(findall(fh, "-property", "FontSize"), "FontSize", 8);
set(findall(fh, "Type", "line"), "LineWidth", 1, "MarkerSize", 3);
exportgraphics(fh, fullfile(outDir, "step_time_vs_speed.pdf"), "ContentType", "vector");

%% Step time table (ms)
T_time = table(vxSimArray(:), 'VariableNames', "vxSim");
for d = vxDesignArray
    f = sprintf("vxDes%d", d);
    Sl = stepTimeStats(results.(f).runs_lti);
    Sv = stepTimeStats(results.(f).runs_ltv);
    T_time.("LTI" + d + "_med") = Sl(:, 1);
    T_time.("LTI" + d + "_p99") = Sl(:, 2);
    T_time.("LTI" + d + "_max") = Sl(:, 3);
    T_time.("LTV" + d + "_med") = Sv(:, 1);
    T_time.("LTV" + d + "_p99") = Sv(:, 2);
    T_time.("LTV" + d + "_max") = Sv(:, 3);
end
fprintf("\nController step time (ms), Ts = %g ms:\n", 1e3 * params.Ts);
disp(T_time);
writetable(T_time, fullfile(outDir, "step_times.csv"));


%% Combined Plot Across All tested Design Velocities
metrics = ["RMSe", "maxErr", "lag95", "maxAy"];
ylabs   = ["RMS e_y (m)", "max |e_y| (m)", "lag_{95} (s)", "max |a_y| (m/s^2)"];
cols    = lines(numel(vxDesignArray));

figure("Name", "Metric trends - vxDes 15 vs 25");
tiledlayout(2, 2, "TileSpacing", "compact");

for m = 1:numel(metrics)
    ax = nexttile; hold(ax, "on"); grid(ax, "on");
    for k = 1:numel(vxDesignArray)
        d = vxDesignArray(k);
        f = sprintf("vxDes%d", d);
        Tl = results.(f).T_lti;  Tv = results.(f).T_ltv;
        plot(ax, Tl.vxSim, Tl.(metrics(m)), "-o",  "Color", cols(k,:), ...
            "DisplayName", sprintf("LTI, v_{design}=%d", d));
        plot(ax, Tv.vxSim, Tv.(metrics(m)), "--s", "Color", cols(k,:), ...
            "DisplayName", sprintf("LTV, v_{design}=%d", d));
    end
    xlabel(ax, "v_{x,sim} (m/s)"); ylabel(ax, ylabs(m));
end
legend(ax, "Location", "eastoutside")
fig = gcf;
fig.Units = "inches";
fig.Position = [1 1 7.16 4.5];          % width x height in inches
set(findall(fig, "-property", "FontSize"), "FontSize", 8);
set(findall(fig, "Type", "line"), "LineWidth", 1.2, "MarkerSize", 4);
exportgraphics(fig, "metric_sweep_lti_ltv_15_25.pdf", "ContentType", "vector");