function Tensegrity = AffineTransform3D(Tensegrity)

% get node 1, 2, and 3.
node1 = Tensegrity.Node(1,:);
node2 = Tensegrity.Node(2,:);
node3 = Tensegrity.Node(3,:);

% Firstly, node-1 as a reference point. Then, the unit vector vx is the unit
% direction vector point from node-1 to node-2, and the unit vector vz is the
% unit direction vector obtained by the corss of the vector point from node-1 to
% node-2 and the vector point from node-1 to node-3. Finally, the unit vector vy
% is the vector obtained by the cross of the vector vz and vx. 
v12 = node2-node1; v12 = v12/norm(v12);
v13 = node3-node1; v13 = v13/norm(v13);
vx = v12;vz = cross(v12,v13);
vz = vz/norm(vz);vy = cross(vz,vx);

translation = node1;
rotation = [vx;vy;vz]^-1;
n_node = Tensegrity.n_node;
% translation and rotaion Tensegrity.
Tensegrity.Node = Tensegrity.Node - repmat(translation,n_node,1);
Tensegrity.Node = Tensegrity.Node*rotation;

end