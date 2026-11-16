clear; clc; close all;
mpcverbosity('off'); % Prevent mpc object outputting missing weights before they are set
addpath("functions\")

%% System Level Definitions
params = paramDefs(); % Initialise with vxDesign = 15 m/s
[~,s_fun] = systemDefinition(params);
r = repmat(s_fun.gF(params.xLin, params.uLin).', params.Np, 1);   % reference held fixed across prediction horizon

%% Initialise the MPC Object based on the linearisation parameters
[mpcobj, xmpc] = buildUnconstrainedMPC(params, s_fun);

%% LTI-15 - Initialised once at the start, based on vxDesign = 15 m/s
ltiStepMPC = @(x, uPrev) stepMPC(false, mpcobj, xmpc, s_fun, x, uPrev, r);
[kxLTI, kuLTI] = gainMatrices(ltiStepMPC, params.xLin, params.uLin);

%% LTI-25 - same construction, designed at 25 m/s
% Reusing r is fine: the controller is unconstrained, so the reference only
% adds a constant to the move, which cancels in the finite differences.
params25 = paramDefs("vxDesign", 25);
[mpcobj25, xmpc25] = buildUnconstrainedMPC(params25, s_fun);
lti25StepMPC = @(x, uPrev) stepMPC(false, mpcobj25, xmpc25, s_fun, x, uPrev, r);
[kxLTI25, kuLTI25] = gainMatrices(lti25StepMPC, params25.xLin, params25.uLin);

%% Sweep the car speed
vList = linspace(5, 85, 100);
keep = 2:6;                           % drop X position (always eigenvalue 1)
rhoLTI   = zeros(size(vList));
rhoLTI25 = zeros(size(vList));
rhoLTV   = zeros(size(vList));

for i = 1:numel(vList)
    pSim = paramDefs("vxSim",vList(i));
    xCar = pSim.x0;

    % The car at the speed it is actually driving (same for all controllers)
    [A, B, plantK, nomK] = linearisedCar(s_fun, xCar, pSim.u0, pSim.Ts);
    A = A(keep, keep);
    B = B(keep, :);

    % LTI: fixed design-speed gains against the car at v
    rhoLTI(i)   = closedLoopRho(A, B, kxLTI(:, keep),   kuLTI);
    rhoLTI25(i) = closedLoopRho(A, B, kxLTI25(:, keep), kuLTI25);

    % LTV Gain measured at v
    ltvStepMPC = @(x, uPrev) stepMPC(true, mpcobj, xmpc, s_fun, x, uPrev, r, plantK, nomK);
    [kxLTV, kuLTV] = gainMatrices(ltvStepMPC, xCar, pSim.u0);
    rhoLTV(i) = closedLoopRho(A, B, kxLTV(:, keep), kuLTV);
end

%% Results
reportCrossing('LTI-15', vList, rhoLTI);
reportCrossing('LTI-25', vList, rhoLTI25);
reportCrossing('LTV',    vList, rhoLTV);

fh = figure('Name', 'Closed-loop stability vs speed');

plot(vList, rhoLTI,   '-',  'DisplayName', 'LTI-15');
hold on;
plot(vList, rhoLTI25, '-',  'DisplayName', 'LTI-25');
plot(vList, rhoLTV,   '--', 'DisplayName', 'LTV');
yline(1, 'k:', 'HandleVisibility', 'off');
xline([params.vxDesign params25.vxDesign], ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
grid on;
xlabel('Car speed v_x (m/s)');
ylabel('Largest |eigenvalue|');
legend('Location', 'northwest');

% Size for a single IEEE column, then export to "Figures and Results"
fh.Units = 'inches';
fh.Position = [1 1 3.5 2.6];
set(findall(fh, '-property', 'FontSize'), 'FontSize', 8);
set(findall(fh, 'Type', 'line'), 'LineWidth', 1);

outDir = fullfile(fileparts(mfilename('fullpath')), 'Figures and Results');
if ~isfolder(outDir), mkdir(outDir); end
exportgraphics(fh, fullfile(outDir, 'stability_vs_speed.pdf'), 'ContentType', 'vector');

function reportCrossing(name, vList, rho)
k = find(rho >= 1, 1);
if isempty(k)
    fprintf('%s: stable at every speed tested (largest |eig| = %.3f)\n', name, max(rho));
elseif k == 1
    fprintf('%s: unstable at the lowest speed tested\n', name);
else
    vStar = interp1(rho(k-1:k), vList(k-1:k), 1);
    fprintf('%s: becomes unstable at about %.2f m/s\n', name, vStar);
end
end
