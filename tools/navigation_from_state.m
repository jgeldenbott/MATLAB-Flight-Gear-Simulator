function [x_nav] = navigation_from_state(X)
%% Extract the state vector
x1 = X(1);
x2 = X(2);
x3 = X(3);
x7 = X(7);
x8 = X(8);
x9 = X(9);

V_b = [x1;x2;x3];

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
];

Cbv = Cb2*C21*C1v;
Cvb = Cbv';

x_nav = Cvb*V_b;
x_nav(3) = -x_nav(3);

% expand the vector
% V_N = x_nav(1);
% V_E = x_nav(2);
% h_dot = -x_nav(3);
end

