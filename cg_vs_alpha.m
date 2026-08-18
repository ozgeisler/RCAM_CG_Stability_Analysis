clear; 
clc;

cbar = 6.6;

%% Fill these in from your Part 2 straight-and-level trim
Va_trim = 85;        % trim airspeed (m/s)
u2_trim = -0.0959;   % trim stabilizer deflection dT (rad)

alpha_deg = -5:0.5:15;
Xcg_frac  = [-0.3 -0.25 -0.2 -0.15 -0.1 0 0.1 0.2 0.3 0.4 0.5];

Cm = zeros(length(Xcg_frac), length(alpha_deg));

for i = 1:length(Xcg_frac)
    Xcg = Xcg_frac(i)*cbar;
    for j = 1:length(alpha_deg)
        alpha = alpha_deg(j)*pi/180;
        u = Va_trim*cos(alpha);
        w = Va_trim*sin(alpha);
        X = [u; 0; w; 0; 0; 0; 0; 0; 0];
        U = [0; u2_trim; 0; 0; 0];
        [~, Cmcg_w] = RCAM_Model_CG(X, U, Xcg);
        Cm(i,j) = Cmcg_w;
    end
end

figure;
hold on;
grid on; 
box on;
colors = jet(length(Xcg_frac));
for i = 1:length(Xcg_frac)
    plot(alpha_deg, Cm(i,:), 'LineWidth', 1.5, 'Color', colors(i,:), ...
        'DisplayName', sprintf('X_{CG} = %.2g', Xcg_frac(i)));
end
xlabel('\alpha (deg)');
ylabel('Pitching Coefficient in Wind Axis');
title('Pitching Coefficient in Wind Axis vs. Angle of Attack for Various X_{CG} Locations');
legend('Location','eastoutside');
xlim([-5 15]);