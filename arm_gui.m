clc, clear
close all  
L0=0.6; L1=67.50; L2=53.94; L3=25.00; L4=126.20; L5=148.57; % [mm] -> [m] 
d1 = 30; d2=25.00;
%pi=3.141592;
th1=pi/4; th2=pi/4; th3=pi/4;
%th1=0;th2=0;th3=0;

% R -> L (X->Z)
% dhparams = [0, pi/2, 0, 0;
%             0, 0, L1,  th1;
%             0, 0, 0, -pi/2;
%             L3, -pi/2, L2, th2;
%             0,-pi/2, 0, pi/2;
%             -L4,-pi/2, 0, (pi/2)+th3;
%             L5, 0, 0, 0];

dhparams = [0, 0, -L0, 0;
            0, pi/2, d1, th1;
            0, 0, L2, -pi/2;
            d2, -pi/2, 0, th2;
            0,-pi/2, -L4, pi/2;
            0,-pi/2, 0, 0;
            0, 0, 0, (pi/2)+th3;
            L5, 0, 0, 0];


iiwa_MDH = robotics.RigidBodyTree;

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');

setFixedTransform(jnt1,dhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(iiwa_MDH,body1,'base')

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


addBody(iiwa_MDH,body2,'body1')
addBody(iiwa_MDH,body3,'body2')
addBody(iiwa_MDH,body4,'body3')
addBody(iiwa_MDH,body5,'body4')
addBody(iiwa_MDH,body6,'body5')
addBody(iiwa_MDH,body7,'body6')
addBody(iiwa_MDH,body8,'body7')

figure;

show(iiwa_MDH);

%robot = importrobot('iiwa14.urdf');
%robot.show('visuals','off');