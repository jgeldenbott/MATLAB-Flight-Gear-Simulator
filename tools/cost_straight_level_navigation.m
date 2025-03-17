function [cost] = cost_straight_level_navigation(Z)

X = Z(1:12);
U = Z(13:17);

xdot = RCAM_model_navigation(X, U);
Va = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
gam = atan2(-X(12), (X(10)^2+X(11)^2)^(1/2));
Xi = pi/2 - atan2(X(10), X(11));

Q = [
    xdot(1:9);
    Va-85;
    xdot(12);
    X(2);
    X(7);
    X(9);
];

H = diag(ones(1, 14));

cost = Q'*H*Q;
end

