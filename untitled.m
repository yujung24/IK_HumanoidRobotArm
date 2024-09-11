
syms th1 th2 th3 h0 L1 L2 L3 L4 d
pi=sym(pi);
syms R1 R2 R3 R4 R5 R6 R7 R8 R9 x y z 
A = [1 0 0 0; 0 0 -1 L1; 0 1 0 0; 0 0 0 1];
B=A^(-1);

C=[R1 R2 R3 x; R4 R5 R6 y;R7 R8 R9 z;0 0 0 1];

D = B*C;

MT01 = DHmodified( 0,      0,  0,  0);
MT12 = DHmodified( L1,  0,   0,  th1);
MT23 = DHmodified(L2,      0,   d,  th2);
MT34 = DHmodified(L3,      -pi/2,   0,    th3);
MT4e = DHmodified(L4,      0,   0,    0);


MT0e = MT01 * MT12 * MT23 * MT34 * MT4e

MT14 = (MT01^(-1))*MT0e;

T01 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1;]*[1 0 0 0; 0 1 0 L1; 0 0 1 0; 0 0 0 1];
T12 = [cos(th1) 0 sin(th1) 0; 0 1 0 0; -sin(th1) 0 cos(th1) -d; 0 0 0 1;]*[1 0 0 0; 0 1 0 L2; 0 0 1 0; 0 0 0 1];
T23 = [1 0 0 0; 0 cos(th2) -sin(th2) 0; 0 sin(th2) cos(th2) 0; 0 0 0 1;]*[1 0 0 0; 0 1 0 L3; 0 0 1 0; 0 0 0 1];
T34 = [cos(th3) -sin(th3) 0 0; sin(th3) cos(th3) 0 0; 0 0 1 0; 0 0 0 1;]*[1 0 0 0; 0 0 1 L4; 0 -1 0 0; 0 0 0 1];

T04 = T01*T12*T23*T34;

T14 = (T01^(-1))*T04;

T14 = (T01^(-1))*[R1 R2 R3 x; R4 R5 R6 y; R7 R8 R9 z; 0 0 0 1];

function [T] = DHmodified(a, alpha, d, th)
    [T] = [           cos(th)             -sin(th)             0               a;
           sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
           sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
                            0                    0             0               1];
end