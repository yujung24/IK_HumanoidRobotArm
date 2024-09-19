clc, clear
close all  

L1 = 67.50; L2 = 53.94; L3 = 126.20; L4 = 148.57;
d1 = 45.50; d2 = 25.00;

%syms x1 y1 z1
x1 = 0.98;
y1 = 0.05;
z1 = 0.01;

L1=L1/100;L2=L2/100;L3=L3/100;L4=L4/100;
d1=d1/100;d2=d2/100;

syms th1 th2 th3
%th1=pi/4; th2=pi/4; th3=pi/4;

dhparams = [0, 0, -d1, 0;
            0, pi/2, L1, th1;
            0, 0, L2, -pi/2;
            d2, -pi/2, 0, th2;
            0,-pi/2, 0, pi/2;
            0,-pi/2, 0, pi/2;
            L3, 0, 0, th3;
            L4, 0, 0, 0];


srcirc24_MDH = robotics.RigidBodyTree;

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');

setFixedTransform(jnt1,dhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(srcirc24_MDH,body1,'base')

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','fixed');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','fixed');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','fixed');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
body8 = robotics.RigidBody('body8');
jnt8 = robotics.Joint('jnt8','fixed');

setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');
setFixedTransform(jnt7,dhparams(7,:),'mdh');
setFixedTransform(jnt8,dhparams(7,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
body8.Joint = jnt8;


addBody(srcirc24_MDH,body2,'body1')
addBody(srcirc24_MDH,body3,'body2')
addBody(srcirc24_MDH,body4,'body3')
addBody(srcirc24_MDH,body5,'body4')
addBody(srcirc24_MDH,body6,'body5')
addBody(srcirc24_MDH,body7,'body6')
addBody(srcirc24_MDH,body8,'body7')


gik = generalizedInverseKinematics("RigidBodyTree",srcirc24_MDH)

value = inverseKinematics("RigidBodyTree",srcirc24_MDH)


dhparams = [0, 0, -d1, 0;
            0, pi/2, L1, th1;
            0, 0, L2, -pi/2;
            d2, -pi/2, 0, th2;
            0,-pi/2, 0, pi/2;
            0,-pi/2, 0, pi/2;
            L3, 0, 0, th3;
            L4, 0, 0, 0];

for i=1:size(dhparams,1)
    MT = DHmodified(dhparams(i,:));
    if i==1
        MT0e=MT;
    else
        MT0e = MT0e*MT;
    end
    
end

i=1;
period = 0.03;
dt=0.01;


th_M = [th1 th2 th3];
J_M(i,j) = diff(MT0e(i,4), th_M(j));
J_M_inv = J_M^(-1);
lim1 = 0; lim2 = pi/2;

J0 = subs(J_M,[th1;th2;th3],q0);
J0_inv = J0^(-1);

q0 = [pi/4; pi/4; pi/4];

for tt = 0:dt:period+1
    if i==1
        q_old = q0;
        J_inv = J0_inv;
        X_new = X1;
        X_old = X0;
    else
        q_old = q_new;
        %X_old = X_new;
        %X_new = subs(MT0e(1:3, 4), [th1;th2;th3], q_old);
        X_old = subs(MT0e(1:3, 4), [th1;th2;th3], q_old); %FK
        X_new = X1;
        J = subs(J_M,[th1;th2;th3],q_old);
    end
    q_new = J_inv*(X_new-X_old) + q_old;
    if X_new == X_old
        disp("arrive!");
        break;
    end
    if(th3 == lim1 | th3 == lim2)
        disp("not valid position!");
        break;
    end
    i=i+1;
end

X0 = MT0e(1:3,4)%subs(MT0e(1:3, 4), th1, pi/4);
X1 = [0.05; 0.02; 0.01]; 

Ts=0.01;
wayPoints = [X0(1:2) X1(1:2)] ;
timestamps = [0 0.3]; % 0 period
timevec = timestamps(1):Ts:timestamps(end);

[q,qd,qdd,~] = cubicpolytraj(wayPoints,timestamps,timevec);
%figure;

%show(srcirc24_MDH);
%gui = interactiveRigidBodyTree(srcirc24_MDH,MarkerScaleFactor=1.0);

plot(timevec, q)
hold all
plot(timestamps, wayPoints, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off


function [T] = DHmodified(mdhparams)

a= mdhparams(1);
alpha= mdhparams(2); 
d= mdhparams(3);
th= mdhparams(4);
    [T] = [           cos(th)             -sin(th)             0               a;
           sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
           sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
                            0                    0             0               1];
end
