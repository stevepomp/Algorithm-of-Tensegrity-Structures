function PlotTimeHistory(NDResult, idofs, option)
% plot position, velocity, and acceleration time history curve.

time = zeros(numel(NDResult),1);
var = zeros(numel(NDResult),numel(idofs));
if (strcmp(option,'position'))
    for i = 1:numel(NDResult)
        time(i) = NDResult(i).time;
        for j = 1:numel(idofs)
            var(i,j) = NDResult(i).q(idofs(j));
        end
    end
elseif (strcmp(option,'velocity'))
    for i = 1:numel(NDResult)
        time(i) = NDResult(i).time;
        for j = 1:numel(idofs)
            var(i,j) = NDResult(i).qt(idofs(j));
        end
    end
elseif (strcmp(option,'acceleration'))
    for i = 1:numel(NDResult)
        time(i) = NDResult(i).time;
        for j = 1:numel(idofs)
            var(i,j) = NDResult(i).qtt(idofs(j));
        end
    end
end

figure; clf; hold on;
line_colors = ['r','b','g','c','m','b','y'];
for j = 1:numel(idofs)
    plot(time, var(:,j), 'Color', line_colors(j), 'LineWidth', 3);
end
set(gca,'color','none');
set(gcf,'color','none');
grid on;

end