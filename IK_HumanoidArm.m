clc, clear
close all


pi = sym(pi);


syms th1 th2 th3        % motor angle
syms L1 L2 L3 L4 L5     % Link length
syms x0 y0 z0           % X0 : initial task space [x0 y0 z0]
syms x1 y1 z1           % X1 : target task space [x1 y1 z1]

syms T s1 s2 s3 c1 c2 c3 % transfer 

q0 = [pi/4; pi/4; pi/4]; %initial angle : 512

x=0.5; y=0.7; z=0.1;     % target coordinate (x,y,z)


%% ========================== setting ============================


%---------------------- Homogeneous Matrix ----------------------%

MT01 = DHmodified(0, pi/2, 0, 0);
MT12 = DHmodified(0, 0, L1,  th1);
MT23 = DHmodified(0, 0, 0, -pi/2);
MT34 = DHmodified(L3, -pi/2, L2, th2);
MT45 = DHmodified(0,-pi/2, 0, pi/2);
MT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th3);
MT6e = DHmodified(L5, 0, 0, 0);

MT0e = simplify(MT01 * MT12 * MT23 * MT34 * MT45 * MT56 * MT6e);



%-------------- calculate task space (Xdot = X1-X0) --------------%

% substitution theta
%X0 = [x0; y0; z0];
X0 = subs(MT0e(1:3, 4), [th1;th2;th3], [pi/4;pi/4;pi/4]); % initial X : [x0, y0, z0]

X1 = [x1; y1; z1]; % target X

X_target = [x; y; z];

%------------------------ Jacobian Matrix ------------------------%

% partial derivative
for i = 1:3
    th_M = [th1 th2 th3];
    for j = 1:3
        J(i,j) = simplify([diff(MT0e(i,4), th_M(j))]);
    end
end

J_inv = simplify(J^(-1));   % J inverse Matrix

limit = (det(J));
lim1 = 0; lim2 = pi/2;      % 나중에 solve로 바꾸기



%% ========================= first moving =========================

J0 = subs(J,[th1;th2;th3],q0);

J0_inv = J0^(-1);
q1 = J0*(X1-X0) + q0;


%q_new = qdot + q_old;

X_old = X1;
q_old = q1;


X_new = subs(MT0e(1:3, 4), [th1; th2; th3], q_old);     % Joinst space -> Task space (FK)
J_inv_old = subs(J_inv, [th1;th2;th3], q_old);
q_new = J_inv_old*(X_new - X_old) + q_old;


%% ====================== continuous moving =======================

% FK : X0, -> q0 -> q1 -> J^(-1)



assume(th3 > 0 & th3 < pi/2); % singularity



while(X_target~=X_new)
    
    X_new = subs(MT0e(1:3, 4), [th1; th2; th3], q_old);     % Joinst space -> Task space (FK)
    J_inv_old = subs(J_inv, [th1;th2;th3],q_old);
    q_new = J_inv_old*(X_new - X_old) + q_old;

    if(th3 == lim1 | th3 == lim2)
        break;
    end

end



%% ============================== plot ==============================






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






function [T] = DHmodified(a, alpha, d, th)


    [T] = [           cos(th)             -sin(th)             0               a;
           sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
           sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
                            0                    0             0               1];
end