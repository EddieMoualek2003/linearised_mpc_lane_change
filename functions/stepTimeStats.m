function S = stepTimeStats(runs)
% Returns [median, 99th percentile, max] step time in ms, one row per run
    keys = fieldnames(runs);
    S = zeros(numel(keys), 3);
    for j = 1:numel(keys)
        ts = runs.(keys{j}).StepTime(2:end) * 1e3;   % drop warm-up step
        S(j, :) = [median(ts), prctile(ts, 99), max(ts)];
    end
end