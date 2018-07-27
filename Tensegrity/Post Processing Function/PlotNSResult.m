function PlotNSResult(iNSResult, Model)
% Plot the i-th Result of Nonlinear Statics Solver.

LabelOn = iNSResult.LabelOn;
radius = iNSResult.radius;

n_elem = Model.n_elem;
e_mat = zeros(n_elem,6);
for i = 1:n_elem
    node1 = iNSResult.node(Model.Elem(i,1),:);
    node2 = iNSResult.node(Model.Elem(i,2),:);
    e_mat(i,1:6) = [node1,node2];
end

figure;clf;set(gcf, 'renderer', 'zbuffer');hold on;%

light('position',[0,-10,5],'style','infinite');
lighting gouraud;material shiny;

indx = [1,4];X = e_mat(:,indx);
indy = [2,5];Y = e_mat(:,indy);
indz = [3,6];Z = e_mat(:,indz);

Xmax = max(iNSResult.node(:,1));Xmin = min(iNSResult.node(:,1));
Ymax = max(iNSResult.node(:,2));Ymin = min(iNSResult.node(:,2));
Zmax = max(iNSResult.node(:,3));Zmin = min(iNSResult.node(:,3));
Dmax = max([Xmax-Xmin,Ymax-Ymin,Zmax-Zmin]);scaling = 1/Dmax;

X = scaling*X;Y = scaling*Y;Z = scaling*Z;

% % % plot nodes
% method 1
[XS,YS,ZS] = sphere(20);rSphere = radius;
colormap([0.2,0.2,0.2;0.2,0.2,0.2]);
for i = 1:Model.n_node
    surf(scaling*iNSResult.node(i,1)+rSphere*XS, ...
        scaling*iNSResult.node(i,2)+rSphere*YS, ...
        scaling*iNSResult.node(i,3)+rSphere*ZS);
    if (LabelOn)
        text(scaling*iNSResult.node(i,1), scaling*iNSResult.node(i,2), ...
            scaling*iNSResult.node(i,3), ['N' num2str(i)], 'FontSize', 16, 'Color', 'b');
    end
end
shading interp;

strut_lines = ['k','k','k','k','k'];
% % % plot struts
for i = 1:Model.n_strut
    ie = Model.strut_index(i,1);is=Model.strut_group(i,1);
    plot3(X(ie,:), Y(ie,:), Z(ie,:), strut_lines(is), 'LineWidth', 6);
    if (LabelOn)
        text(mean(X(ie,:)), mean(Y(ie,:)), mean(Z(ie,:)), ...
            ['S' num2str(i)], 'FontSize', 16, 'Color', 'm');
    end
end

cable_lines = ['r','b','g','c','m','b','y'];%cable_lines = ['r','r','r','r','r','r','r'];
% % % plot cables
for i = 1:Model.n_cable
    ie = Model.cable_index(i,1);ic=Model.cable_group(i,1);
    plot3(X(ie,:), Y(ie,:), Z(ie,:), cable_lines(ic), 'LineWidth', 3);
    if (LabelOn)
        text(mean(X(ie,:)), mean(Y(ie,:)), mean(Z(ie,:)), ...
            ['C' num2str(i)], 'FontSize', 16, 'Color', 'm');
    end
end

set(gca,'position',[0 0 1 1]);
set(gca,'color','none');
set(gcf,'color','w');
axis off;axis image;axis equal;

end