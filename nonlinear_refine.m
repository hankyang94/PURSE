function [R,t] = nonlinear_refine(A,B,R0,t0)
x0 = [R0(:);t0];

cost = @(x) closest_pose(x,x0);
nonlcon = @(x) pose_con(x,A,B);


options = optimoptions('fmincon',...
                       'Algorithm','interior-point',... % interior-point, sqp
                       'Display','iter',...
                       'CheckGradients',false,...
                       'MaxIterations',1000,...
                       'SpecifyObjectiveGradient',false,...
                       'SpecifyConstraintGradient',false);
                   
[x,fopt,exitflag,output] = fmincon(cost,x0,[],[],[],[],[],[],nonlcon,options);
fprintf('NLP pose refine: exitflag: %d, constraint violation: %3.2e, first-order opt: %3.2e, cost: %3.8e.\n',exitflag,output.constrviolation,output.firstorderopt,fopt);

if exitflag > 0 || (output.constrviolation < 1e-8 && output.firstorderopt < 1e-3)
    r = x(1:9);
    t = x(10:end);
    R = reshape(r,3,3);
else
    R = [];
    t = [];
end
end


function f = closest_pose(x,x0)
diff = x - x0;
f    = diff' * diff;
end

function [c,ceq] = pose_con(x,A,B)
r = x(1:9);
R = reshape(r,3,3);
c1      = R(:,1);
c2      = R(:,2);
c3      = R(:,3);

% Equality constraints: R in SO(3)
ceq = [c1'*c1 - 1;
       c2'*c2 - 1;
       c3'*c3 - 1;
       c1'*c2;
       c2'*c3;
       c3'*c1;
       cross(c1,c2) - c3;
       cross(c2,c3) - c1;
       cross(c3,c1) - c2];
   
% Inequality constraints
c = [];
for i = 1:size(A,1)
    Ai = squeeze(A(i,:,:));
    c  = [c;x'*Ai*x];
end
slack = 1e-3;
for i = 1:size(B,1)
    Bi = squeeze(B(i,:));
    Bi = Bi(:);
    c  = [c; -Bi'*x + slack ];
end
end