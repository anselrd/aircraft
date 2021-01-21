% define vehicle environment
earth_nowind.g = 9.81;
earth_nowind.Wx = 0;
earth_nowind.Wy = 0;
earth_nowind.Wz = 0;
earth_nowind.rho = 1.225;

% define physical properties
params.m = 0.068;
params.d = 0.060;
params.I = diag([6.8e-5 9.2e-5 1.35e-4]);
params.thruster_locs = [-params.d/sqrt(2)  params.d/sqrt(2) 0  1; ...
                        -params.d/sqrt(2) -params.d/sqrt(2) 0 -1; ...
                         params.d/sqrt(2) -params.d/sqrt(2) 0  1; ...
                         params.d/sqrt(2)  params.d/sqrt(2) 0 -1];
params.k = 0.0024;
params.eta = 1e-3;
params.zeta = 3e-3;
params.alpha = 2e-6;
params.beta = 1e-7;

% define a trim state
hover.u = 0;
hover.v = 0;
hover.w = 0;
hover.phi = 0;
hover.theta = 0;
hover.psi = 0;
hover.p = 0;
hover.q = 0;
hover.r = 0;
hover.f = params.m*earth_nowind.g/4*ones(4,1);

% create quad object
rolling_spider.trim = hover;
rolling_spider.params = params;
rolling_spider.environment = earth_nowind;

% default controller can hover
rolling_spider.controller = @(x) params.m*earth_nowind.g/4*ones(4,1);

% save it
save('rolling_spider','rolling_spider')