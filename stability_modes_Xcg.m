% ===== Part e: longitudinal eigenvalues vs Xcg =====
clear
clc
close all

temp = load('linear_models_vs_Xcg.mat');
A_all = temp.A_all;
X_cg = temp.X_cg;
N = length(X_cg);

lon_idx = [1 3 5 8];   % u, w, q, theta

figure; hold on; grid on; box on;
colors = jet(N);
legend_entries = cell(1,N);

fprintf('Xcg/cbar    max(Re(eig_lon))    stability\n');
for i = 1:N
    A_lon = A_all(lon_idx, lon_idx, i);
    e = eig(A_lon);

    plot(real(e), imag(e), 'o', 'MarkerSize',8, ...
        'MarkerFaceColor',colors(i,:), 'MarkerEdgeColor','k');
    legend_entries{i} = sprintf('X_{CG} = %.2g', X_cg(i));

    max_re = max(real(e));
    if max_re < 0
        stability = 'stable';
    else
        stability = 'UNSTABLE';
    end
    fprintf('%8.3f      %10.5f        %s\n', X_cg(i), max_re, stability);
end

plot([0 0], ylim, 'k--');
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Longitudinal Eigenvalues vs. X_{CG} (Descending Flight Trim)');
legend(legend_entries, 'Location','eastoutside');

for i = 1:N
    A_lon = A_all([1 3 5 8],[1 3 5 8],i);
    e = eig(A_lon);
    fprintf('Xcg = %.2f: ', X_cg(i));
    fprintf('%.4f%+.4fi  ', [real(e) imag(e)]');
    fprintf('\n');
end