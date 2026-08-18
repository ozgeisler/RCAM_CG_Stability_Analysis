function [F0] = cost_descend(Z, Xcg)
X = Z(1:9);
U = Z(10:14);
xdot = RCAM_Model_CG(X,U,Xcg);

theta = X(8);
Va = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3),X(1));
gam = theta - alpha;

Q = [xdot;
    Va - 67.6;
    gam - (-3*pi/180);
    X(2);
    X(7);
    X(9)];

H = diag([ ...
    1 1 1 ...       % udot vdot wdot
    10 10 10 ...    % pdot qdot rdot
    100 100 100 ... % phidot thetadot psidot
    10 ...           % Va
    1e4 ...          % gamma
    100 ...          % v
    100 ...          % phi
    100]);           % psi
F0 = Q'*H*Q;
end