function [E, A_P, B_P] = ImplicitLinmod(FUN, XDOT0, X0, U0, DXDOT, DX, DU)

% obtain number of states and controls
n = length(XDOT0);
m = length(U0);

%-----------------------CALCULATE E MATRIX---------------------------------
% initialize E matrix
E = zeros(n,n);

% fill in each element of the matrix individually
for i = 1:n
    for j = 1:n
        % obtain magnitude of perturbation for the current index
        dxdot = DXDOT(i,j);

        % define the perturbation vector
        xdot_plus = XDOT0;
        xdot_minus = XDOT0;

        xdot_plus(j) = xdot_plus(j) + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot;

        % calculate F at the linearization point for the plus case
        F = feval(FUN, xdot_plus, X0, U0);
        F_plus_keep = F(i);


        % calculate F at the linearization point for the minus case
        F = feval(FUN, xdot_minus, X0, U0);
        F_minus_keep = F(i);

        % Calculate E for the current index
        E(i,j) = (F_plus_keep - F_minus_keep) / (2*dxdot);
    end
end

%-----------------------CALCULATE A_P MATRIX-------------------------------
A_P = zeros(n,n);

for i = 1:n
    for j = 1:n
        % obtain magnitude of perturbation for the current index
        dx = DX(i,j);

        % define the perturbation vector
        x_plus = X0;
        x_minus = X0;

        x_plus(j) = x_plus(j) + dx;
        x_minus(j) = x_minus(j) - dx;

        % calculate F at the linearization point for the plus case
        F = feval(FUN, XDOT0, x_plus, U0);
        F_plus_keep = F(i);


        % calculate F at the linearization point for the minus case
        F = feval(FUN, XDOT0, x_minus, U0);
        F_minus_keep = F(i);

        % Calculate E for the current index
        A_P(i,j) = (F_plus_keep - F_minus_keep) / (2*dx);
    end
end

%-----------------------CALCULATE B_P MATRIX-------------------------------
B_P = zeros(n,m);

for i = 1:n
    for j = 1:m
        % obtain magnitude of perturbation for the current index
        du = DU(i,j);

        % define the perturbation vector
        u_plus = U0;
        u_minus = U0;

        u_plus(j) = u_plus(j) + du;
        u_minus(j) = u_minus(j) - du;

        % calculate F at the linearization point for the plus case
        F = feval(FUN, XDOT0, X0, u_plus);
        F_plus_keep = F(i);


        % calculate F at the linearization point for the minus case
        F = feval(FUN, XDOT0, X0, u_minus);
        F_minus_keep = F(i);

        % Calculate E for the current index
        B_P(i,j) = (F_plus_keep - F_minus_keep) / (2*du);
    end
end

end

