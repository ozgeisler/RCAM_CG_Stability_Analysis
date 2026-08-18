clear
clc
close all

cbar = 6.6;
Xcg_frac = [-0.3 -0.25 -0.2 -0.15 -0.1 0 0.1 0.2 0.3 0.4 0.5];
N = length(Xcg_frac);

% Initialize Z guess
initilatization = 1;
if(initilatization==0)
    Z_guess = zeros(14,1);
    Z_guess(1) = 67.6;
   
else
    temp = load('trim_values_descend_vs_Xcg');
    Z_guess = [temp.XStar_all(:,1);
        temp.UStar_all(:,1)];
end

XStar_all = zeros(9,N);
UStar_all = zeros(5,N);
VaStar_all = zeros(1,N);
gammaStar_all = zeros(1,N);
vStar_all = zeros(1,N);
f0_all      = zeros(N);
XdotStar_all = zeros(9,N); 
for k = 1:N
    Xcg = Xcg_frac(k)*cbar;

    % Solve unconstrained optimization problem
    [ZStar,f0] = fminsearch(@(Z) cost_descend(Z,Xcg),Z_guess,...
        optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000));

    XStar = ZStar(1:9);
    UStar = ZStar(10:14);

    % Verify that this satisfies the constraints
    XdotStar = RCAM_Model_CG(XStar,UStar,Xcg)
    VaStar = sqrt(XStar(1)^2 + XStar(2)^2 + XStar(3)^2)
    gammaStar = XStar(8) - atan2(XStar(3),XStar(1))
    vStar = XStar(2)
   

    XStar_all(:,k) = XStar;
    UStar_all(:,k) = UStar;
    VaStar_all(k) = VaStar;
    gammaStar_all(k) = gammaStar;
    vStar_all(k) = vStar;
    f0_all (k)      = f0;
    XdotStar_all(:,k)  = XdotStar; 

    Z_guess = ZStar;   % warm-start next Xcg with this solution
end

save('trim_values_descend_vs_Xcg','Xcg_frac','XStar_all','UStar_all',...
    'VaStar_all','gammaStar_all','vStar_all',"XdotStar_all")