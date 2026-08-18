function [E,A_P,B_P] = ImplicitLinMod(MY_FUN,XDOTo,Xo,Uo,DXDOT,DX,DU)

%Obtain the number of states and controls
n = length(XDOTo);
m = length (Uo);

%--------------------CALCULATE E MATRIX-------------------------
%Initialize the E matrix
E = zeros(n,n);

% fill in each element of the matrix individually
for i =1:n
    for j =1:n
        % Obtain the magnitude of the perturbation to use
        dxdot = DXDOT(i,j);

        % Define the perturbation vector.The current column detrmines which
        %element of xdot we are perturbing
        xdot_plus = XDOTo;
        xdot_minus = XDOTo;

        xdot_plus(j) = xdot_plus(j) + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot;

        % Calculate F(row)(xdot_plus,x0,u0)
        F = feval(MY_FUN,xdot_plus,Xo,Uo);
        F_plus_keep = F(i);

        % Calculate F(row)(xdot_minus,x0,u0)
        F = feval(MY_FUN,xdot_minus,Xo,Uo);
        F_minus_keep = F(i);

        %CALCULATE E(row,col)
        E(i,j) = (F_plus_keep - F_minus_keep)/(2*dxdot);
    end
end

%-----------------------------Calculate A_P Matrix-------------------------
A_P = zeros(n,n);
for i = 1:n
    for j = 1:n
      
        dx = DX(i,j);

        % Define the perturbation vector.The current column detrmines which
        %element of xdot we are perturbing
        x_plus = Xo;
        x_minus = Xo;

        x_plus(j) = x_plus(j) + dx;
        x_minus(j) = x_minus(j) - dx;

        % Calculate F(row)(xdot_plus,x0,u0)
        F = feval(MY_FUN,XDOTo,x_plus,Uo);
        F_plus_keep = F(i);

        % Calculate F(row)(xdot_minus,x0,u0)
        F = feval(MY_FUN,XDOTo,x_minus,Uo);
        F_minus_keep = F(i);

        %CALCULATE E(row,col)
        A_P(i,j) = (F_plus_keep - F_minus_keep)/(2*dx);
    end
end

%-----------------CALCULATE THE B_P MATRIX----------------
B_P = zeros(n,m);
for i = 1:n
    for j = 1:m
        du = DU(i,j);

        % Define the perturbation vector.The current column detrmines which
        %element of x we are perturbing
        u_plus = Uo;
        u_minus = Uo;

        u_plus(j) = u_plus(j) + du;
        u_minus(j) = u_minus(j) - du;

        % Calculate F(row)
        F = feval(MY_FUN,XDOTo,Xo,u_plus);
        F_plus_keep = F(i);

        % Calculate F(row)
        F = feval(MY_FUN,XDOTo,Xo,u_minus);
        F_minus_keep = F(i);

        %CALCULATE E(row,col)
        B_P(i,j) = (F_plus_keep - F_minus_keep)/(2*du);
    end
end


