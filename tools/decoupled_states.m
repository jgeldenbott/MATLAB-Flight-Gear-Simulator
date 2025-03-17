P = zeros(9,9);

P(1,1) = 1;
P(2,3) = 1;
P(3,5) = 1;
P(4,8) = 1;
P(5,2) = 1;
P(6,4) = 1;
P(7,6) = 1;
P(8,7) = 1;
P(9,9) = 1;

T = inv(P);

A_tilde = inv(T) * A * T;
B_tilde = inv(T) * B;

disp("A_tilde: ")
disp(A_tilde)
disp("B_tilde: ")
disp(B_tilde)
