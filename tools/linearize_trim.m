% do not clear data because this is called from 'RCAM_LQR.m'

X0 = X_bar;
U0 = U_bar;

XDOT0 = zeros(9,1);

dxdot_matrix = 10e-6*ones(9,9);
dx_matrix = 10e-6*ones(9,9);
du_matrix = 10e-6*ones(9,5);

[E, A_P, B_P] = ImplicitLinmod(@RCAM_model_implicit, XDOT0, X0, U0, ...
    dxdot_matrix, dx_matrix, du_matrix);

% calculate A and B matricess
A = -inv(E)*A_P;
B = -inv(E)*B_P;

disp('A: ')
disp(A)
disp('B: ')
disp(B)