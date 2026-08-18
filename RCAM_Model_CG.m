function [XDOT, Cmcg_w] = RCAM_Model_CG(X,U,Xcg_in)
%----------------------------------CONSTANTS---------------------------
% Nominal Vehicle constants
m = 120000;              % Aircraft Total mass (kg)
cbar = 6.6;              % Mean Aerodynamic  Chord (m)
lt = 24.8;               % Distance by AC of tail and body (m)
S = 260;                 % Wing planform area (m^2)
St = 64;                 % Tail planform area (m^2)


if nargin < 3 || isempty(Xcg_in)
    Xcg = 0.23*cbar;      % default x position of CoG in Fm (m)
else
    Xcg = Xcg_in;          % allow Xcg to be swept externally (Part 3a)
end
Ycg = 0;                 % y position of CoG in Fm (m)
Zcg = 0.10*cbar;        % z position of CoG in Fm
Xac = 0.12*cbar;         % x position of aerodynamic center in Fm (m)
Yac = 0;                 % y position of aerodynamic center in Fm (m)
Zac = 0;                 % z position of aerodynamic center in Fm (m)
% Engine Inputs
Umax = 120000*9.81;       % Maximum thrust provided by one engine (N)
Xapt1 = 0;                % x position of engine 1 force in Fm (m)
Yapt1 = -7.94;            % y position of engine 1 force in Fm (m)
Zapt1 = -1.9;             % z position of engine 1 force in Fm (m)
Xapt2 = 0;                % x position of engine 2 force in Fm (m)
Yapt2 = 7.94;             % y position of engine 2 force in Fm (m)
Zapt2 = -1.9;             % z position of engine 2 force in Fm (m)
%Other constants
rho = 1.225;                 % Air density (kg/m^3)
g = 9.81;                    % Gravitational acceleration (m/s^2)
depsda = 0.25;               % Change in downwash w.r.t alpha (rad/rad)
alpha_L0 = -11.5*(pi/180);   % Zero lift angle of attack (rad)
n = 5.5;                     % Slope of linear region of lift slope
a3 = -768.5;                 % Coefficient of alpha^3
a2 = 609.2;                  % Coefficient of aloha^2
a1 = -155.2;                 % Coefficient of alpha^2
a0 = 15.212;                 % Coefficient of alpha^0 (modificated to get rid of discontinuity)
alpha_switch = 14.5*(pi/180); % alpha where lift slope goes from linear to non-linear
%---------------------------------State Vector-------------------------
%Extract the State vector
x1 = X(1);          % u
x2 = X(2);          % v
x3 = X(3);          % w
x4 = X(4);          % p
x5 = X(5);          % q
x6 = X(6);          % r
x7 = X(7);          % phi
x8 = X(8);          % theta
x9 = X(9);          % psi
% Extract the Control Vector
u1 = U(1);          % d_A (aileron)
u2 = U(2);          % d_T (stabilizer)
u3 = U(3);          % d_R (rudder)
u4 = U(4);          % d_th1 (throttle 1)
u5 = U(5);          % d_th2 (throttle 2)
%-----------------Intermediate Variables--------------------------
% Calculate air speed
Va = sqrt(x1^2 + x2^2 + x3^2);
% Calculate alpha and beta
alpha = atan2(x3,x1);
beta = asin(x2/Va);
% Calculate the dynamic pressure
Q = 1/2*rho*(Va^2);
% Defining the vectors wbe_b and v_b
wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];
%----------------------------AerodynamicCoefficients---------------------------------------
%Calculate CL_wb
if alpha<=alpha_switch
    CL_wb = n*(alpha-alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end
% Calculate Cl_t
epsilon = depsda*(alpha -alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*(lt/Va);
CL_t = 3.1*alpha_t * (St/S);
% Total Lift force
CL = CL_wb + CL_t;
% Total Drag Force(neglecting tail)
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;
% Total SideForce
CY = -1.6*beta + 0.24*u3;
%-----------------Dimensional Aerodynamic Forces---------------------------
FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];
%Rotate this forces to F_b (body axis)
C_bs = [cos(alpha) 0 -sin(alpha);
    0 1 0;
    sin(alpha) 0 cos(alpha)];
FA_b = C_bs*FA_s;
%-----------------Aerodynamic Moment Coefficient About AC-------------------------------
% Calculate the moments ibn Fb.Defining eta,dCMdx and dCMdu
eta11 = -1.4*beta;
eta21 = -0.59 - (3.1*(St*lt)/(S*cbar))*(alpha-epsilon);
eta31 = (1-alpha*(180/(15*pi)))*beta;
eta = [eta11;
      eta21;
      eta31];
dCMdx = (cbar/Va)*[-11 0 5;
               0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
               1.7 0 -11.5];
dCMdu = [-0.6 0 0.22;
    0 (-3.1*(St*lt)/(S*cbar)) 0;
    0 0 -0.63];
% Now calculate CM = [Cl;Cm;Cn] about Aerodynamic center in Fb
CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];
%-------------Aerodynamic Moment about AC-------------------
% Normalize to an aerodynamic moment
MAac_b = CMac_b*Q*S*cbar;
%--------------------------Aerodynamic Moment About CG-----------
% Tranfer Moment equation to CG
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg_b = MAac_b + cross(FA_b,rcg_b - rac_b);

%--- Part 3a output: pitching moment coefficient about CG, wind axis ---
Cm_b = MAcg_b / (Q*S*cbar);   % [Cl; Cm; Cn] about cg, body axis (aero only)

L_alpha = [cos(alpha) 0 sin(alpha);
           0           1 0;
          -sin(alpha)  0 cos(alpha)];      % body -> stability

L_beta  = [cos(beta)  sin(beta) 0;
          -sin(beta)  cos(beta) 0;
           0           0        1];         % stability -> wind

Cw = L_beta * L_alpha * Cm_b;               % [Cl''; Cm''; Cn''] in wind axes

Cmcg_w = Cw(2);

%--------------------------------Engine Force and Moment--------------------
F1 = u4*Umax;
F2 = u5*Umax;
% Assuming that engine thrust is aligned with Fb,we have
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];
FE_b = FE1_b + FE2_b;
% Engine Moment due to offset of engine thrust from CoG
mew1 = [Xcg-Xapt1;
     Yapt1 - Ycg;
     Zcg - Zapt1];
mew2 = [Xcg-Xapt2;
     Yapt2 - Ycg;
     Zcg - Zapt2];
MEcg1_b = cross(mew1,FE1_b);
MEcg2_b = cross(mew2,FE2_b);
MEcg_b = MEcg1_b + MEcg2_b;
%------------------Gravity EFfects----------------
% Calculate gravitational forces in the body frame.It causes no moment aout
% Cog.
g_b = [-g*sin(x8);
    g*cos(x8)*sin(x7);
    g*cos(x8)*cos(x7)];
Fg_b = m*g_b;
%----------------------------------STATE DERIVATIVES-------------
% Inertia Matrix
Ib = m*[40.07 0 -2.0923;
        0 64 0 ;
        -2.0923 0 99.92];
% Inverse of inertia matrix
invIb = (1/m) *[0.0249836 0 0.000523151;
                 0 0.015625 0;
                 0.000523151 0 0.010019];
% Form F_b (all the forces in Fb) and calculate udot,vdot,wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);
% Form Mcg_b (all moment about CoG in Fb) and calculate pdot,qdot,rdot
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b,Ib*wbe_b));
% Calculate phidot,thetadot, and psidot
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
    0 cos(x7) -sin(x7);
    0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
x7to9dot = H_phi*wbe_b;
% Place in first order form
XDOT = [x1to3dot
        x4to6dot
        x7to9dot];
end




