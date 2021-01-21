%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% define vehicle environment %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stdatm40000ft.g = 9.81;
stdatm40000ft.Wx = 0;
stdatm40000ft.Wy = 0;
stdatm40000ft.Wz = 0;
stdatm40000ft.alt = convlength(40000,'ft','m');
stdatm40000ft.rho = convdensity(5.909e-4,'slug/ft^3','kg/m^3');

stdatm20000ft.g = 9.81;
stdatm20000ft.Wx = 0;
stdatm20000ft.Wy = 0;
stdatm20000ft.Wz = 0;
stdatm20000ft.alt = convlength(20000,'ft','m');
stdatm20000ft.rho = convdensity(1.2673e-3,'slug/ft^3','kg/m^3');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% define physical properties %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.W = 2.83176e6;
params.m = params.W/stdatm40000ft.g;
params.S = 5500*convlength(1,'ft','m')^2;
params.b = convlength(195.68,'ft','m');
params.AR = (params.b)^2/params.S;
params.cbar = convlength(27.31,'ft','m');
params.h = 0.25;
params.IB = diag([1.83e7 3.31e7 4.97e7]) + diag(-1.56e6,2) + diag(-1.56e6,-2);
params.xi = deg2rad(-2.4);
params.I = Rot(2,-6.8*pi/180)*params.IB*Rot(2,6.8*pi/180);
params.CD = 0.043;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% define a trim state %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
trimflight.V = 518;
trimflight.M = 0.8;
trimflight.u = trimflight.V;
trimflight.v = 0;
trimflight.w = 0;
trimflight.phi = 0;
trimflight.theta = 0;
trimflight.psi = 0;
trimflight.p = 0;
trimflight.q = 0;
trimflight.r = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% stability derivatives %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% longitudinal
dimless.Cxu = -0.1080;
dimless.Cxa = 0.2193;
dimless.Cxq = 0;
dimless.Cxad = 0;
dimless.Czu = -0.1060;
dimless.Cza = -4.920;
dimless.Czq = -5.921;
dimless.Czad = 5.896;
dimless.Cmu = 0.1043;
dimless.Cma = -1.023;
dimless.Cmq = -23.92;
dimless.Cmad = -6.314;
dimless.Cw0 = params.W/(0.5*stdatm40000ft.rho*trimflight.V^2*params.S);

% lateral
dimless.Cyb = -0.8771;
dimless.Cyp = 0;
dimless.Cyr = 0;
dimless.Clb = -0.2797;
dimless.Clp = -0.3295;
dimless.Clr = 0.304;
dimless.Cnb = 0.1946;
dimless.Cnp = -0.04073;
dimless.Cnr = -0.2737;

% control
dimless.Cxde = -3.818e-6;
dimless.Czde = -0.3648;
dimless.Cmde = -1.444;

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% default controller %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
default_controller = @(x) deal(0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% create aircraft object %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Boeing_747.trim = trimflight;
Boeing_747.params = params;
Boeing_747.environment = stdatm20000ft;
Boeing_747.coefs = dimless;
Boeing_747.controller = default_controller;

%%%%%%%%%%%%%%%
%%% save it %%%
%%%%%%%%%%%%%%%
save('Boeing_747','Boeing_747')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% clear unnecessary variables %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear default_controller dimless params stdatm40000ft trimflight

% Drag terms
% aircraft_parameters.e = 0.9515; %[-], AVL, Oswald's efficiency factor
% aircraft_parameters.K = 1/(pi*(aircraft_parameters.b^2/aircraft_parameters.S)*aircraft_parameters.e); %drag polar coefficient (CD = CDpa + kCL^2)
% aircraft_parameters.CDmin = 0.04; %[-] This is the parasite drag. Total drag is combination of parasite drag and induced drag.

% % Engine parameters
% aircraft_parameters.Sprop = 0.0507; %[m^2]
% aircraft_parameters.Cprop = 1;
% aircraft_parameters.kmotor = 25;
% 
% % Zero angle of attack aerodynamic forces and moments
% dimless.CL0 = 0.2176;
% dimless.Cm0 = -0.0165;
% 
% dimless.CY0 = 0;
% dimless.Cl0 = 0;
% dimless.Cn0 = 0;
% 
% %Control surface deflection parameters
% % Elevator
% dimless.CLde = 0.00869*180/pi;
% dimless.Cmde = -0.033837*180/pi;
%   
% % Aileron
% dimless.CYda = -0.001057*180/pi;
% dimless.Clda = -0.004875*180/pi;
% dimless.Cnda = -0.000194*180/pi;
%  
% % Rudder
% dimless.CYdr = 0.00317*180/pi;
% dimless.Cldr = 0.00007*180/pi;
% dimless.Cndr = -0.00074*180/pi;