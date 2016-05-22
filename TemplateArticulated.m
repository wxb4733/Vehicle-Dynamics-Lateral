%% Template Articulated
% Template script for the simulation of articulated vehicle.
%
%% Code start

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

import VehicleDynamicsLateral.*   % Import package Vehicle Dynamics

%% Integration parameters
%

% Simulation time
T = 7;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

% Initial conditions
X0 = 0;                     % Initial tractor CG horizontal position [m]
Y0 = 0;                     % Initial tractor CG vertical position [m]
PSI0 = 0;                   % Initial tractor yaw angle [rad]
PHI0 = 0;                   % Initial articulation angle [rad]
V0 = 20;                  % Initial tractor CG velocity [m/s]
ALPHAT0 = 0.3;              % Initial tractor side slip angle [rad]
dPSI0 = 0.25;               % Initial tractor yaw rate [rad/s]
dPHI0 = dPSI0;              % Initial articulation rate [rad/s]

x0 = [X0 Y0 PSI0 PHI0 V0 ALPHAT0 dPSI0 dPHI0];

%% Default models and parameters
% Defining the model of the vehicle without passing any argument the default parameters and models are used.

System = VehicleDynamicsLateral.VehicleArticulatedNonlinear4DOF;

%% Integration
% Integration using mass matrix. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>

% Configuring integration options so that it takes into consideration the mass matrix
options = odeset('Mass',@System.MassMatrix);

[TOUT,XOUT] = ode45(@(t, estados) System.Model(t, estados),TSPAN,x0,options);

%% Post integration
%

% Retrieving states
XT = XOUT(:,1);             % Tractor CG horizontal position [m]
YT = XOUT(:,2);             % Tractor CG vertical position [m]
PSI = XOUT(:,3);            % Tractor yaw angle [rad]
PHI = XOUT(:,4);            % Articulation angle [rad]
VEL = XOUT(:,5);            % Tractor CG velocity [m/s]
ALPHAT = XOUT(:,6);         % Tractor side slip angle [rad]
dPSI = XOUT(:,7);           % Tractor yaw rate [rad/s]
dPHI = XOUT(:,8);           % Articulation rate [rad/s]

%% Results
% Details: <Graphics.html Graphics.m>

G = VehicleDynamicsLateral.Graphics(System);

%%
% Trajectory
%

G.Frame([XT YT PSI dPSI VEL ALPHAT PHI dPHI],TOUT,1);

%%
% Animation
%

G.Animation([XT YT PSI dPSI VEL ALPHAT PHI dPHI],TOUT,0);

%%
%
% <<illustrations/AnimationArticulated.gif>>
%
%% See Also
%
% <index.html Index> | <TemplateSimple.html TemplateSimple>
%
