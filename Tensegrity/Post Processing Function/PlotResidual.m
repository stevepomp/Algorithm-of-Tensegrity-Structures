function PlotResidual(Tensegrity)

figure; hold on;

plot(Tensegrity.Residual(:,1), Tensegrity.Residual(:,2), 'r-.', 'LineWidth', 3);%

xlabel('Number of iterations', 'FontSize',16, 'FontName', 'Times');
ylabel('log||F||_2', 'FontSize',16, 'FontName', 'Times');
axis([0, Tensegrity.Residual(end,1), 1.1*min(Tensegrity.Residual(:,2)), max(Tensegrity.Residual(:,2))]);
set(gca,'FontSize',16, 'FontName', 'Times');
set(gca,'Color','none');set(gcf,'Color','none');

set(gca, 'Box', 'on');

% h = legend('Analytical solution', 'Numerical solution','FontSize',16);
% set(h,'box','off');

end