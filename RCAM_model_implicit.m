function [FVAL] = RCAM_model_implicit(XDOT,X,U,Xcg)

FVAL = RCAM_Model_CG(X,U,Xcg) - XDOT;
end
