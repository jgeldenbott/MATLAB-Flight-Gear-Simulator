function [FVAL] = RCAM_model_implicit_navigation(XDOT, X, U)
FVAL = RCAM_model_navigation(X, U) - XDOT;
end

