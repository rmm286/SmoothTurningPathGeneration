function [stateOut,controlOut] = PlanPath()
%PlanPath Summary of this function goes here
%   Detailed explanation goes here

clear all;
%% Provide bounds for all values in both problems
t0 = 0;
tf = 1000;

x0 = 0;
y0 = 0;
th0 = pi/2.0;
k0 = 0;
kDot0 = 0;
v0 = 1;
vDot0 = 0;

xf = 1;
yf = 0;
thf = (-1/2.0)*pi; %note, solver is sensitive to positive or negative theta
kf = 0;
kDotf = 0;
vf = 1;
vDotf = 0;

xMin = -10;
xMax = +10;
yMin = -2;
yMax = +6;
thMin = -1000;
thMax = +1000;

kMin = -0.785;
kMax = +0.785;
kDotMin = -1;
kDotMax = +1;
kDDotMin = -0.1;
kDDotMax = +0.1;

vMin = -0.9;
vMax = +1;
vDotMin = -1;
vDotMax = +1;
vDDotMin = -0.5;
vDDotMax = +0.5;

%% -----------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = t0;
bounds.phase.finaltime.upper = tf;

bounds.phase.initialstate.lower = [x0, y0, th0, k0, v0];
bounds.phase.initialstate.upper = [x0, y0, th0, k0, v0];
bounds.phase.finalstate.lower = [xf, yf, thf, kf, vf];
bounds.phase.finalstate.upper = [xf, yf, thf, kf, vf];

bounds.phase.state.upper = [xMax, yMax, thMax, kMax, vMax];
bounds.phase.state.lower = [xMin, yMin, thMin, kMin, vMin];

bounds.phase.control.lower = [kDotMin, vDotMin];
bounds.phase.control.upper = [kDotMax, vDotMax];

%% -----------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%

guess.phase.time = [t0; tf];
guess.phase.state = [x0, y0, th0, k0, v0; xf, yf, thf, kf, vf];
guess.phase.control = [0, 0; 0, 0;];

%% -----------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method = 'hp-LiuRao-Legendre';

%% -----------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Smooth-Turning-Path';
setup.functions.continuous = @smoothTurningContinuous;
setup.functions.endpoint = @smoothTurningEndpoint;
setup.displaylevel = 0;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.scales.method = 'automatic-hybridUpdate';
setup.nlp.solver = 'ipopt';
setup.nlp.snoptoptions.tolerance = 1e-10;
setup.nlp.snoptoptions.maxiterations = 20000;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance = 1e-10;
setup.derivatives.supplier = 'sparseFD';
setup.derivatives.derivativelevel = 'second';
setup.method = 'RPM-Differentiation';
setup.derivatives.stepsize = 1e-10;

%% -----------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);

%% set-up variables for second iteration 
clear bounds guess mesh setup

guess.phase.time = output.result.solution.phase.time;
guess.phase.state = [output.result.solution.phase.state, zeros(size(output.result.solution.phase.state,1),2)];
guess.phase.control = zeros(size(output.result.solution.phase.control,1),2);

%% Call second iteration
clear output

%% ------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = t0;
bounds.phase.finaltime.upper = tf;

bounds.phase.initialstate.lower = [x0, y0, th0, k0, v0, kDot0, vDot0];
bounds.phase.initialstate.upper = [x0, y0, th0, k0, v0, kDot0, vDot0];
bounds.phase.finalstate.lower = [xf, yf, thf, kf, vf, kDotf, vDotf];
bounds.phase.finalstate.upper = [xf, yf, thf, kf, vf, kDotf, vDotf];

bounds.phase.state.upper = [xMax, yMax, thMax, kMax, vMax, kDotMax, vDotMax];
bounds.phase.state.lower = [xMin, yMin, thMin, kMin, vMin, kDotMin, vDotMin];

bounds.phase.control.upper = [kDDotMax, vDDotMax];
bounds.phase.control.lower = [kDDotMin, vDDotMin];

%% ------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method = 'hp-LiuRao-Legendre';

%% ------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.name = 'Smooth-Turning-Path-double-dot';
setup.functions.continuous = @smoothTurningContinuousDDot;
setup.functions.endpoint = @smoothTurningEndpoint;
setup.displaylevel = 0;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
%setup.scales.method = 'automatic-hybridUpdate';
setup.nlp.solver = 'ipopt';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance = 1e-10;
setup.nlp.ipoptoptions.maxiterations = 2000;
setup.derivatives.supplier = 'sparseFD';
setup.derivatives.derivativelevel = 'first';
setup.method = 'RPM-Differentiation';
setup.derivatives.stepsize = 1e-10;

%% ------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
stateOut = output.result.solution.phase.state;
controlOut = output.result.solution.phase.control;

end
