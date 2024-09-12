clc, clear
close all


pi = sym(pi);


syms th1 th2 th3        % motor angle
syms L1 L2 L3 L4 L5     % Link length
syms x0 y0 z0           % X0 : initial task space [x0 y0 z0]
syms x1 y1 z1           % X1 : target task space [x1 y1 z1]

syms T s1 s2 s3 c1 c2 c3 % transfer 

q0 = [pi/4; pi/4; pi/4]; %initial angle : 512


%% ============================ setting ============================

%----------------------- Homogeneous Matrix -----------------------%

MT01 = DHmodified(0, pi/2, 0, 0);
MT12 = DHmodified(0, 0, L1,  th1);
MT23 = DHmodified(0, 0, 0, -pi/2);
MT34 = DHmodified(L3, -pi/2, L2, th2);
MT45 = DHmodified(0,-pi/2, 0, pi/2);
MT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th3);
MT6e = DHmodified(L5, 0, 0, 0);

MT0e = simplify(MT01 * MT12 * MT23 * MT34 * MT45 * MT56 * MT6e);



%--------------- calculate task space (Xdot = X1-X0) ---------------%

% substitution theta
X0 = [x0; y0; z0];
% X0 = subs(MT0e(1:3, 4), th1, pi/4); % initial X : [x0, y0, z0]
% X0 = subs(X0, th2, pi/4);
% X0 = subs(X0, th3, pi/4);

%X1 = MT0e(1:3, 4);   % target X : [x1, y1, z1]
% OR
X1 = [x1; y1; z1]; 

Xdot = X1-X0;

%------------------------- Jacobian Matrix -------------------------%

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




%% =========================== first step ===========================

J0 = subs(J,[th1;th2;th3],q0);

J0_inv = J0^(-1);
q1 = J0*(X1-X0) + q0;



%q_new = qdot - q_old;





%% ======================== continuous moving ========================


q_new = simplify(J_inv*(X1-X0) + q_old);
assume(th3 > 0 & th3 < pi/2); % singularity



while(q_new==q_old)
    q_new = qdot - q_old;

    if(th3 == lim1 | th3 == lim2)
    end

end



%% =============================== plot ===============================

J
q0
q1


%====================== Substitution equation ======================%

%q1 = subs(q1, L2*cos(th3)*sin(th2) - L3*sin(th3) + L4*cos(th3)*sin(th2), T);

% substitution sin cos
q1 = subs(q1, sin(th1), s1);
q1 = subs(q1, sin(th2), s2);
q1 = subs(q1, sin(th3), s3);
q1 = subs(q1, cos(th1), c1);
q1 = subs(q1, cos(th2), c2);
q1 = subs(q1, cos(th3), c3);


q1 = subs(q1, L2*c3*s2 - L3*s3 + L4*c3*s2, T);


syms K K2 C C2 C3 C4    % C : Constant value
q1 = subs(q1, (L2*c1 + L4*c1 + L3*s1 + c3*s1*s2*L5 + L5*c1*s3)*c2*s3, K);
q1 = subs(q1, L2*s1 - L3*c1 + L4*s1 + L5*s1*s3 - L5*c1*c3*s2, K2);
q1 = subs(q1, (2^(1/2)*L2)/2 + (2^(1/2)*L3)/2 + (2^(1/2)*L4)/2 + (2^(1/2)*L5)/4, C);
q1 = subs(q1, (2^(1/2)*L2)/2 - (2^(1/2)*L3)/2 + (2^(1/2)*L4)/2 - (2^(1/2)*L5)/4 + L5/2 - z1, C2);
q1 = subs(q1, L1 + L5/2 + y1, C3);
q1 = subs(q1, C + L5/2 - x1, C4);




%div = simplify(q1(2)/q1(1))

%simplify(atan2(q1(2), q1(1)))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






function [T] = DHmodified(a, alpha, d, th)


    [T] = [           cos(th)             -sin(th)             0               a;
           sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
           sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
                            0                    0             0               1];
end