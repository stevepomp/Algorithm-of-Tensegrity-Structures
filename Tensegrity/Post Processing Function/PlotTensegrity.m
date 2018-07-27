function PlotTensegrity(Tensegrity)
% Plot Tensegrity Prototype.

LabelOn = Tensegrity.LabelOn;
n_elem = Tensegrity.n_elem;
e_mat = zeros(n_elem,6);
for i = 1:n_elem
    node1 = Tensegrity.Node(Tensegrity.Elem(i,1),:);
    node2 = Tensegrity.Node(Tensegrity.Elem(i,2),:);
    e_mat(i,1:6) = [node1,node2];
end

figure;clf;set(gcf, 'renderer', 'zbuffer');hold on;%

light('position',[0,-10,5],'style','infinite');
lighting gouraud;material shiny;

indx = [1,4];X = e_mat(:,indx);
indy = [2,5];Y = e_mat(:,indy);
indz = [3,6];Z = e_mat(:,indz);

Xmax = max(Tensegrity.Node(:,1));Xmin = min(Tensegrity.Node(:,1));
Ymax = max(Tensegrity.Node(:,2));Ymin = min(Tensegrity.Node(:,2));
Zmax = max(Tensegrity.Node(:,3));Zmin = min(Tensegrity.Node(:,3));
Dmax = max([Xmax-Xmin,Ymax-Ymin,Zmax-Zmin]);scaling = 1/Dmax;

X = scaling*X;Y = scaling*Y;Z = scaling*Z;

% % % plot nodes
% method 1
[XS,YS,ZS] = sphere(20);rSphere = 0.01;
colormap([0.2,0.2,0.2;0.2,0.2,0.2]);
for i = 1:Tensegrity.n_node
    surf(scaling*Tensegrity.Node(i,1)+rSphere*XS, ...
        scaling*Tensegrity.Node(i,2)+rSphere*YS, ...
        scaling*Tensegrity.Node(i,3)+rSphere*ZS);
    if (LabelOn)
        text(scaling*Tensegrity.Node(i,1), scaling*Tensegrity.Node(i,2), ...
            scaling*Tensegrity.Node(i,3), ['N' num2str(i)], 'FontSize', 16, 'Color', 'b');
    end
end
shading interp;

strut_lines = ['k','k','k','k','k'];
% % % plot struts
for i = 1:Tensegrity.n_strut
    ie = Tensegrity.strut_index(i,1);is=Tensegrity.strut_group(i,1);
    plot3(X(ie,:), Y(ie,:), Z(ie,:), strut_lines(is), 'LineWidth', 6);
    if (LabelOn)
        text(mean(X(ie,:)), mean(Y(ie,:)), mean(Z(ie,:)), ...
            ['S' num2str(i)], 'FontSize', 16, 'Color', 'm');
    end
end

cable_lines = ['r','b','g','c','m','b','y'];%cable_lines = ['r','r','r','r','r','r','r'];
% % % plot cables
for i = 1:Tensegrity.n_cable
    ie = Tensegrity.cable_index(i,1);ic=Tensegrity.cable_group(i,1);
    plot3(X(ie,:), Y(ie,:), Z(ie,:), cable_lines(ic), 'LineWidth', 3);
    if (LabelOn)
        text(mean(X(ie,:)), mean(Y(ie,:)), mean(Z(ie,:)), ...
            ['C' num2str(i)], 'FontSize', 16, 'Color', 'm');
    end
end

set(gca,'position',[0 0 1 1]);
set(gca,'color','none');
set(gcf,'color','w');
% set(gcf,'InvertHardCopy','on');
axis off;axis image;axis equal;

end
