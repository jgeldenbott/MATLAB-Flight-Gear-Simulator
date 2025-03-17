function [xdot] = RCAM_model_navigation(X, U)
%% Extract the state vector
x1 = X(1);
x2 = X(2);
x3 = X(3);
x4 = X(4);
x5 = X(5);
x6 = X(6);
x7 = X(7);
x8 = X(8);
x9 = X(9);

u1 = U(1);          % d_A   : aileron
u2 = U(2);          % d_T   : stabilizer
u3 = U(3);          % d_R   : rudder
u4 = U(4);          % d_th1 : throttle 1
u5 = U(5);          % d_th2 : throttle 2

%% Define constants
m = 120000;

cbar = 6.6;         % Mean cord
lt = 24.8;          % Distance from AC of tail to AC of body (m)
S = 260;            % wing planform area (m^2)
St = 64;            % tail planform area (m^2)

Xcg = 0.23*cbar;    % x position of CG in Fm (m)
Ycg = 0;            % y position of CG in Fm (m)
Zcg = 0.10*cbar;    % z position of CG in Fm (m)

Xac = 0.12*cbar;    % x position of AC in Fm (m)
Yac = 0;            % y position of AC in Fm (m)
Zac = 0;            % z position of AC in Fm (m)

% Engine constants
Xapt1 = 0;          % x position of engine 1 in Fm (m)
Yapt1 = -7.94;      % y position of engine 1 in Fm (m)
Zapt1 = -1.9;       % z position of engine 1 in Fm (m)

Xapt2 = 0;          % x position of engine 2 in Fm (m)
Yapt2 = 7.94;       % y position of engine 2 in Fm (m)
Zapt2 = -1.9;       % z position of engine 2 in Fm (m)

% other constants
rho = 1.225;        % air density (kg/m^3)
g = 9.81;           % gravity (m/s^2)
depsda = 0.25;      % change in downwash w.r.t alpha (rad/rad)
alpha_L0 = deg2rad(-11.5);      % zero lift angle of attack (rad)
n = 5.5;            % linear lift slope
a3 = -768.5;        % alpha^3 coefficient
a2 = 609.2;         % alpha^2 coefficient
a1 = -155.2;        % alpha^1 coefficient
a0 = 15.212;        % alpha^0 coefficient
alpha_switch = deg2rad(14.5);   % linear -> nonlinear lift transition

%% 1. Control limits and saturation
u1min = deg2rad(-25);
u1max = deg2rad(25);

u2min = deg2rad(-25);
u2max = deg2rad(10);

u3min = deg2rad(-30);
u3max = deg2rad(30);

u4min = deg2rad(0.5);
u4max = deg2rad(10);

u5min = u4min;
u5max = u4max; 

% check to see if values are within the control limits
% u1 = check_control_limit(u1, u1min, u1max);
% u2 = check_control_limit(u2, u2min, u2max);
% u3 = check_control_limit(u3, u3min, u3max);
% u4 = check_control_limit(u4, u4min, u4max);
% u5 = check_control_limit(u5, u5min, u5max);

%% 2. Intermediate variables
% airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

% alpha and beta
alpha = atan2(x3, x1);
beta = asin(x2/Va);

% dynamic pressure
Q = 0.5 * rho * Va^2;

% angular momentum and velocity vectors
V_b = [x1;x2;x3];
wbe_b = [x4;x5;x6];

%% 3. Aerodynamic force coefficients
if alpha <= alpha_switch
    CL_wb = n*(alpha - alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

% tail lift coefficient
epsilon = depsda*(alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t = 3.1*(St/S)*alpha_t;

% total lift force
CL = CL_wb + CL_t;

% total drag force
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

% sideforce
CY = -1.6*beta + 0.24*u3;

%% 4. Dimensional aerodynamic forces
% in stability axis
FA_s = [
    -CD*Q*S;
    CY*Q*S;
    -CL*Q*S
];

% rotate to the body axis
C_bs = [
    cos(alpha) 0 -sin(alpha);
    0 1 0;
    sin(alpha) 0 cos(alpha)
];

FA_b = C_bs * FA_s;

%% 5. Aerodynamic moment coefficients about aerodynamic center
eta1 = -1.4*beta;
eta2 = -0.59 - (3.1*(St*lt)/(S*cbar)) * (alpha - epsilon);
eta3 = (1 - alpha*(180/(15*pi))) * beta;

eta = [
    eta1;
    eta2;
    eta3
];

dCMdx = (cbar/Va)*[
    -11 0 5;
    0 (-4.03 * (St*lt^2)/(S*cbar^2)) 0;
    1.7 0 -11.5
];

dCMdu = [
    -0.6 0 0.22;
    0 (-3.1*(St*lt)/(S*cbar)) 0;
    0 0 -0.63
];

% calculate CM about aerodynamic center in Fb
CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

%% 6. Aerodynamic moment about aerodynamic center
MAac_b = CMac_b*Q*S*cbar;

%% 7. Aerodynamic moment about center of gravity
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg_b = MAac_b + cross(FA_b, rcg_b-rac_b);

%% 8. Engine force and moment
% thrust of each engine
F1 = u4*m*g;
F2 = u5*m*g;

% force vector of each engine
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];

FE_b = FE1_b + FE2_b;

% moment caused by engine offset
mew1 = [
    Xcg - Xapt1;
    Yapt1 - Ycg;
    Zcg - Zapt1
];

mew2 = [
    Xcg - Xapt2;
    Yapt2 - Ycg;
    Zcg - Zapt2
];

MEcg1_b = cross(mew1, FE1_b);
MEcg2_b = cross(mew2, FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;

%% 9. Gravity effects
g_b = [
    -g*sin(x8);
    g*cos(x8)*sin(x7);
    g*cos(x8)*cos(x7)
];

Fg_b = m*g_b;

%% 10. State derivatives
% intertia matrix
Ib = m * [
    40.07 0 -2.0923;
    0 64 0;
    -2.0923 0 99.92
];

% inverse of inertia matrix
invIb = (1/m) * [
    0.0249836 0 0.000523151;
    0 0.015625 0;
    0.000523151 0 0.010019
];

% form F_b
F_b = Fg_b + FE_b + FA_b;

% calculate x(1:3) -- udot, vdot, wdot
x1to3dot = (1/m)*F_b - cross(wbe_b, V_b);

% form Mcg_b
Mcg_b = MAcg_b + MEcg_b;

% calculate x(4:6) -- pdot, qdot, rdot
% x4to6dot = Ib\(Mcg_b - cross(wbe_b, Ib*wbe_b));
x4to6dot = invIb * (Mcg_b - cross(wbe_b, Ib*wbe_b));

% calculate x(7:9) -- phidot, thetadot, psidot
H_phi = [
    1 sin(x7)*tan(x8) cos(x7)*tan(x8);
    0 cos(x7) -sin(x7);
    0 sin(x7)/cos(x8) cos(x7)/cos(x8)
];

x7to9dot = H_phi*wbe_b;

% Navigation Equations
C1v = [
    cos(x9) sin(x9) 0;
    -sin(x9) cos(x9) 0;
    0 0 1;
];

C21 = [
    cos(x8) 0 -sin(x8);
    0 1 0;
    sin(x8) 0 cos(x8);
];

Cb2 = [
    1 0 0;
    0 cos(x7) sin(x7);
    0 -sin(x7) cos(x7);
];% Navigation Equations
C1v = [
    cos(x9) sin(x9) 0;
    -sin(x9) cos(x9) 0;
    0 0 1;
];

C21 = [
    cos(x8) 0 -sin(x8);
    0 1 0;
    sin(x8) 0 cos(x8);
];

Cb2 = [
    1 0 0;
    0 cos(x7) sin(x7);
    0 -sin(x7) cos(x7);
];

Cbv = Cb2*C21*C1v;
Cvb = Cbv';

x10to12dot = Cvb*V_b;

Cbv = Cb2*C21*C1v;
Cvb = Cbv';

x10to12dot = Cvb*V_b;

% place in first order form
xdot = [
    x1to3dot;
    x4to6dot;
    x7to9dot;
    x10to12dot
];
end

