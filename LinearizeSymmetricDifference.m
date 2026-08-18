clear
clc
close all

% Trim variables w.r.t change in CG (Xcg)
temp = load('trim_values_descend_vs_Xcg.mat');
X_cg = temp.Xcg_frac;   % Center of gravity locations (fraction of cbar)
N = length(X_cg);
cbar = 6.6;

% Define the perturbation matrices
dxdot_matrix = 10e-12*ones(9,9);
dx_matrix    = 10e-12*ones(9,9);
du_matrix    = 10e-12*ones(9,5);

A_all = zeros(9,9,N);
B_all = zeros(9,5,N);
E_all = zeros(9,9,N);

for i = 1:N
    Xcg   = X_cg(i)*cbar;
    Xdoto = zeros(9,1);          % trim point, by definition
    Xo    = temp.XStar_all(:,i);
    Uo    = temp.UStar_all(:,i);

    [E,Ap,Bp] = ImplicitLinMod(@(Xdot,X,U) RCAM_model_implicit(Xdot,X,U,Xcg), ...
        Xdoto,Xo,Uo,dxdot_matrix,dx_matrix,du_matrix);

    A = -inv(E)*Ap;
    B = -inv(E)*Bp;

    A_all(:,:,i) = A;
    B_all(:,:,i) = B;
    E_all(:,:,i) = E;
end

save('linear_models_vs_Xcg.mat','X_cg','A_all','B_all','E_all')