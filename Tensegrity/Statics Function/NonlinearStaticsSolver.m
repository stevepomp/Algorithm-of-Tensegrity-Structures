function NSResult = NonlinearStaticsSolver(Model)
% nonlinear statics solver.
% modified Riks method.
% parameters:
% Model.Control.lambda = 0;
% Model.Control.lambdamin = -100;
% Model.Control.lambdamax = 100;
% Model.Control.Dlambda = 0.1;
% Model.Control.nLoadStep = 1000;
% Model.Control.nIteration = 100;
% Model.Control.Nreference = 6;% fixed.
% Model.Control.disp_tolerance = 1.0e-6;
% Model.Control.force_tolerance = 1.0e-6;

lambdabar = Model.Control.lambda;
lambdamin = Model.Control.lambdamin;
lambdamax = Model.Control.lambdamax;
Dlambda0 = Model.Control.Dlambda;
nLoadStep = Model.Control.nLoadStep;
nIteration = Model.Control.nIteration;
Nref = Model.Control.Nreference;
disp_tolerance = Model.Control.disp_tolerance;
force_tolerance = Model.Control.force_tolerance;

neq = Model.ndofs;
Ncur = ones(nLoadStep,1);
Dl = zeros(nLoadStep,1);

lambda = zeros(nIteration,1);
dlambda = zeros(nIteration,1);
dlambda1 = zeros(nIteration,1);
dlambda2 = zeros(nIteration,1);
Dlambda = zeros(nIteration,1);

qbar = zeros(neq,1);
Rref = zeros(neq,1);
Dq = zeros(neq,nIteration);
Dq1 = zeros(neq,nIteration);
Dq2 = zeros(neq,nIteration);
q = zeros(neq,nIteration);
dq = zeros(neq,nIteration);
dq1 = zeros(neq,nIteration);
dq2 = zeros(neq,nIteration);
dqt = zeros(neq,nIteration);
dqs = zeros(neq,nIteration);
grad = zeros(neq,nIteration);

% initialized qbar and calculate the reference force vector.
for i = 1:Model.n_node
    for j = 1:Model.dim
        if (Model.node_dof(i,j)~=0)
            qbar(Model.node_dof(i,j)) = Model.Node(i,j);
            Rref(Model.node_dof(i,j)) = Model.Load(i,j);
        end
    end
end
qbar0 = qbar; Rref2 = (Rref'*Rref);

NSResult(1:nLoadStep) = struct;
% for the first increment, Dlambda(1) need to be choosen.
Dlambda(1) = Dlambda0;
for m = 1:nLoadStep% loop over load step
    i = 1;% iteration step counter.
    imagflag = 0;
    while(1)
        if ( (m==1) && (i==1) && (imagflag==0) )
            sys = AssemblySS(Model, qbar);
            kt = sys.kt;
            Dq(:,1) = Dlambda(1)*(kt\Rref);
            Dl(1) = sqrt(Dq(:,1)'*Dq(:,1)+Dlambda(1)^2*Rref2);
        end
        if ( (m>=2) && (i==1) && (imagflag==0) )
            if(Ncur(m-1)>10)
                Ncur(m-1)=10;
            end
            Dl(m)=Dl(m-1)*sqrt(Nref/Ncur(m-1));
            % this is a very important amendments.
            if(Dl(m)>2*Dl(1))
                Dl(m)=2*Dl(1);
            end
        end
        if( (m>=2) && (i==1) )
            sys = AssemblySS(Model, qbar);
            kt = sys.kt;
            kref = kt\Rref;
            if ( det(kt)>0 )
                Dlambda(1) = Dl(m)/sqrt(kref'*kref+Rref2);
            else
                Dlambda(1) = Dl(m)/sqrt(kref'*kref+Rref2);
            end
            if ( abs(Dlambda(1)<1.0e-6) )
                res = GetResult(Model, qbar);
                NSResult(m).node = res.node;
                NSResult(m).strain = res.strain;
                NSResult(m).stress = res.stress;
                NSResult(m).lambda = lambdabar;
                NSResult(m+1:nLoadStep) = [];
                warning('@nonlinearStatics: step size is too small!'); %#ok<WNTAG>
                return;
            end
            Dq(:,1) = Dlambda(1)*kref;
        end
        if (i==1)
            q(:,1) = qbar(:) + Dq(:,1);
            lambda(1) = lambdabar + Dlambda(1);
            if (m==1)
                qbar = q(:,1);
                lambdabar = lambda(1);
                res = GetResult(Model, qbar);
                NSResult(m).node = res.node;
                NSResult(m).strain = res.strain;
                NSResult(m).stress = res.stress;
                NSResult(m).lambda = lambdabar;
                break;
            else
                i = i + 1;
            end
        end
        if ( (m>=2) && (i>=2) )
            sys = AssemblySS(Model, q(:,i-1));
            kt = sys.kt;
            kp = sys.kp;
            grad(:,i-1) = lambda(i-1)*Rref - kp;
            dqt(:,i-1) = kt\grad(:,i-1);
            dqs(:,i-1) = kt\Rref;
            a = dqs(:,i-1)'*dqs(:,i-1)+Rref2;
            b = (Dq(:,i-1)'+dqt(:,i-1)')*dqs(:,i-1)+Dlambda(i-1)*Rref2;
            c = (Dq(:,i-1)'+dqt(:,i-1)')*(Dq(:,i-1)+dqt(:,i-1))-Dl(m)^2+Dlambda(i-1)^2*Rref2;
            dlambda1(i) = (-b-sqrt(b^2-a*c))/a;
            dlambda2(i) = (-b+sqrt(b^2-a*c))/a;
            if( (imag(dlambda1(i))~=0) && (imag(dlambda2(i))~=0) )
                Dl(m) = Dl(m)/2;
                imagflag = 1;
                i = 1;
            else
                imagflag = 0;
            end
        end
        if( (m>=2) && (i>=2) && (imagflag==0) )
            dq1(:,i)=dqt(:,i-1)+dlambda1(i)*dqs(:,i-1);
            dq2(:,i)=dqt(:,i-1)+dlambda2(i)*dqs(:,i-1);
            Dq1(:,i)=Dq(:,i-1)+dq1(:,i);
            Dq2(:,i)=Dq(:,i-1)+dq2(:,i);
            theta1=Dq(:,i-1)'*Dq1(:,i);
            theta2=Dq(:,i-1)'*Dq2(:,i);
            if( (theta1>0) && (theta2>0) )
                dlambda(i)=-c/(2*b);
                if(abs(dlambda1(i)-dlambda(i))<abs(dlambda2(i)-dlambda(i)))
                    dlambda(i)=dlambda1(i);
                    Dq(:,i)=Dq1(:,i);
                else
                    dlambda(i)=dlambda2(i);
                    Dq(:,i)=Dq2(:,i);
                end
            elseif( (theta1>0) && (theta2<0) )
                dlambda(i)=dlambda1(i);
                Dq(:,i)=Dq1(:,i);
            else%if( (theta1<0) && (theta2>0) )
                dlambda(i)=dlambda2(i);
                Dq(:,i)=Dq2(:,i);
            end
            Dlambda(i)=Dlambda(i-1)+dlambda(i);
            dq(:,i)=Dq(:,i)-Dq(:,i-1);
            q(:,i)=q(:,i-1)+dq(:,i);
            lambda(i)=lambda(i-1)+dlambda(i);
        end
        if( (m>=2) && (imagflag==0))
            qtmp1 = q(:,i-1)-qbar0;
            qtmp2 = q(:,i)-qbar0;
            temp1 = sqrt(qtmp1'*qtmp1);
            temp2 = sqrt(qtmp2'*qtmp2);
            tol_disp = abs( (temp2-temp1)/temp2 );
            tol_force = sqrt(grad(:,i-1)'*grad(:,i-1))/(sqrt(Rref'*Rref));
            if( tol_disp <= disp_tolerance || tol_force < force_tolerance)
                Ncur(m)=i;
                qbar=q(:,i);
                lambdabar=lambda(i);
                res = GetResult(Model, qbar);
                NSResult(m).node = res.node;
                NSResult(m).strain = res.strain;
                NSResult(m).stress = res.stress;
                NSResult(m).lambda = lambdabar;
                break;
            else
                i=i+1;
                if(i>nIteration)
                    NSResult(m:nLoadStep) = [];
                    warning('@nonlinearStatics: fail to converge!'); %#ok<WNTAG>
                    return;
                end
            end
        end
    end% end while.
    if( m==nLoadStep || lambda(i)>=lambdamax || lambda(i)<=lambdamin )
        if (m==nLoadStep)
            disp('# normal termination with m=nLoadStep.');
        elseif (lambda(i)>lambdamax)
            disp('# normal termination with lambda greater than lambdamax.');
        elseif (lambda(i)<lambdamin)
            disp('# normal termination with lambda less than lambdamin.');
        end
        res = GetResult(Model, qbar);
        NSResult(m).node = res.node;
        NSResult(m).strain = res.strain;
        NSResult(m).stress = res.stress;
        NSResult(m).lambda = lambdabar;
        NSResult(m+1:nLoadStep) = [];
        disp('# solution is done!');
        break;
    end
end% end loop over load step.

end