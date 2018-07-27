function [sys] = AssemblySS(Model, qbar)
% Assembly system matrix for statics solver.

neq = Model.ndofs;
sys.kt = zeros(neq,neq);
sys.kp = zeros(neq,1);

for ie = 1:Model.n_strut
    e = Model.strut_index(ie);
    node(1:3) = Model.Node(Model.Elem(e,1),:);
    node(4:6) = Model.Node(Model.Elem(e,2),:);
    eposition = zeros(6,1);
    for i = 1:6
        if (Model.elem_dof(e,i)~=0)
            eposition(i) = qbar(Model.elem_dof(e,i));
        else
            eposition(i) = node(i);
        end
    end
    physical = Model.strut_group_physical(Model.strut_group(ie));
    [kt, kq] = eTruss3n(eposition, physical);
    % assembly
    for i = 1:6
        r = Model.elem_dof(e,i);
        if (r~=0)
            for j = 1:6
                s = Model.elem_dof(e,j);
                if (s~=0)
                    sys.kt(r,s) = sys.kt(r,s) + kt(i,j);
                end
            end
            sys.kp(r) = sys.kp(r) + kq(i);
        end
    end% end assembly
end

for ie = 1:Model.n_cable
    e = Model.cable_index(ie);
    node(1:3) = Model.Node(Model.Elem(e,1),:);
    node(4:6) = Model.Node(Model.Elem(e,2),:);
    eposition = zeros(6,1);
    for i = 1:6
        if (Model.elem_dof(e,i)~=0)
            eposition(i) = qbar(Model.elem_dof(e,i));
        else
            eposition(i) = node(i);
        end
    end
    physical = Model.cable_group_physical(Model.cable_group(ie));
    [kt, kq, isFailed] = eCable3n(eposition, physical);
    if (isFailed)
        warning('MATLAB:paramAmbiguous', '# element (%d) is failed!', e);
    end
    % assembly
    for i = 1:6
        r = Model.elem_dof(e,i);
        if (r~=0)
            for j = 1:6
                s = Model.elem_dof(e,j);
                if (s~=0)
                    sys.kt(r,s) = sys.kt(r,s) + kt(i,j);
                end
            end
            sys.kp(r) = sys.kp(r) + kq(i);
        end
    end% end assembly
end

end