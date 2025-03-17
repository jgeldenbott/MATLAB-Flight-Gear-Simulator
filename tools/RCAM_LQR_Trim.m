% dont clear because this script is called from 'RCAM_LQR.m'
% define Z_guess in the main RCAM_LQR script

f0 = 10;
tol = 1e-6;

while true
    if f0 >= tol
        [Z_bar, f0] = fminsearch('cost_straight_level', Z_guess, ...
            optimset('TolX', 1e-10, 'MaxFunEvals', 100000, 'MaxIter', 100000));
        Z_guess = Z_bar;
    else
        break
    end
end



X_bar = Z_bar(1:9);
U_bar = Z_bar(10:14);

disp('X_bar:')
disp(X_bar)
disp('U_bar')
disp(U_bar)

% check to see if all the conditions are satisfied
disp('xdot_bar')
disp(RCAM_model(X_bar, U_bar))
disp('Va')
disp(sqrt(X_bar(1)^2 + X_bar(2)^2 + X_bar(3)^2))
X_bar(2)
X_bar(7)
X_bar(9)

% save trim_values_straight_level X_bar U_bar

