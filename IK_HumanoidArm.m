clc, clear
close all
pause(0.1)

format short


Timer = timer('StartDelay', 1, 'Period', 0.01, 'ExecutionMode', 'fixedRate');
Timer.TimerFcn = {@timer_callback};

global pi
global period

pi=3.141592;
period = 0.3;

%pi = sym(pi);

%[mm]
L1 = 67.50; L2 = 53.94; L3 = 126.20; L4 = 148.57;
d1 = 45.50; d2 = 25.00;

x1 = 150; y1=-100; z1=-100;

%[m]
% L1=L1/100;L2=L2/100;L3=L3/100;L4=L4/100;
% d1=d1/100;d2=d2/100;

%x1 = 1.5; y1=-1; z1=-1; 


syms th1 th2 th3        % motor angle
%syms L1 L2 L3 L4 L5     % Link length
syms x0 y0 z0           % X0 : initial task space [x0 y0 z0]
%syms x1 y1 z1           % X1 : target task space [x1 y1 z1]

syms T s1 s2 s3 c1 c2 c3 % transfer 


%% =========================== RigidBody ===========================
srcirc24_MDH = robotics.RigidBodyTree;

dhparams = [0, 0, -d1, 0;
            0, pi/2, L1, th1;%q0(1)
            0, 0, L2, -pi/2;
            d2, -pi/2, 0, th2;%q0(2)
            0,-pi/2, 0, pi/2;
            0,-pi/2, 0, pi/2;
            L3, 0, 0, th3;%q0(3)
            L4, 0, 0, 0];
 
srcirc24_MDH = rigidBodyTree;

T= double(subs(dhparams,[th1;th2;th3],[pi/4; pi/4; pi/4]));
% body1 = robotics.RigidBody('body1');
% jnt1 = robotics.Joint('jnt1','fixed');
% body1.Joint = jnt1;
% addBody(srcirc24_MDH,body1,'base')
% 
% body2 = robotics.RigidBody('body2');
% jnt2 = robotics.Joint('jnt2','revolute');
% body2.Joint = jnt2;
% addBody(srcirc24_MDH,body2,'body1')
% 
% body3 = robotics.RigidBody('body3');
% jnt3 = robotics.Joint('jnt3','fixed');
% body3.Joint = jnt3;
% addBody(srcirc24_MDH,body3,'body2')
% 
% body4 = robotics.RigidBody('body4');
% jnt4 = robotics.Joint('jnt4','revolute');
% body4.Joint = jnt4;
% addBody(srcirc24_MDH,body4,'body3')
% 
% body5 = robotics.RigidBody('body5');
% jnt5 = robotics.Joint('jnt5','fixed');
% body5.Joint = jnt5;
% addBody(srcirc24_MDH,body5,'body4')
% 
% body6 = robotics.RigidBody('body6');
% jnt6 = robotics.Joint('jnt6','fixed');
% body6.Joint = jnt6;
% addBody(srcirc24_MDH,body6,'body5')
% 
% body7 = robotics.RigidBody('body7');
% jnt7 = robotics.Joint('jnt7','revolute');
% body7.Joint = jnt7;
% addBody(srcirc24_MDH,body7,'body6')
% 
% body8 = robotics.RigidBody('body8');
% jnt8 = robotics.Joint('jnt8','fixed');
% body8.Joint = jnt8;
% addBody(srcirc24_MDH,body8,'body7')

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');

setFixedTransform(jnt1,T(1,:),'mdh');
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

setFixedTransform(jnt2,T(2,:),'mdh');
setFixedTransform(jnt3,T(3,:),'mdh');
setFixedTransform(jnt4,T(4,:),'mdh');
setFixedTransform(jnt5,T(5,:),'mdh');
setFixedTransform(jnt6,T(6,:),'mdh');
setFixedTransform(jnt7,T(7,:),'mdh');
setFixedTransform(jnt8,T(7,:),'mdh');

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

joint = [jnt1,jnt2,jnt3,jnt4,jnt5,jnt6,jnt7,jnt8];

figure(3);
show(srcirc24_MDH);

%% ============================ setting ============================

%----------------------- Homogeneous Matrix -----------------------%



for i=1:size(dhparams,1)
    MT = DHmodified(dhparams(i,:));
    if i==1
        MT0e=MT;
    else
        MT0e = MT0e*MT;
    end
    
end

% MT01 = DHmodified(dhparams(1,:));
% MT12 = DHmodified(dhparams(2,:));
% MT23 = DHmodified(dhparams(3,:));
% MT34 = DHmodified(dhparams(4,:));
% MT45 = DHmodified(dhparams(5,:));
% MT56 = DHmodified(dhparams(6,:));
% MT6e = DHmodified(dhparams(7,:));
% MT01 = DHmodified(0, pi/2, 0, 0);
% MT12 = DHmodified(0, 0, L1,  th1);
% MT23 = DHmodified(0, 0, 0, -pi/2);
% MT34 = DHmodified(L3, -pi/2, L2, th2);
% MT45 = DHmodified(0,-pi/2, 0, pi/2);
% MT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th3);
% MT6e = DHmodified(L5, 0, 0, 0);
% MT0eq = simplify(MT01 * MT12 * MT23 * MT34 * MT45 * MT56 * MT6e);


%--------------- calculate task space (Xdot = X1-X0) ---------------%

% substitution theta
%X0 = [x0; y0; z0];
X0 = double(subs(MT0e(1:3, 4), [th1;th2;th3], [pi/4;pi/4;pi/4])); % initial X : [x0, y0, z0]
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
        J_M(i,j) = vpa([diff(MT0e(i,4), th_M(j))],16);
    end
end

%start(Timer);
J_M_inv = vpa(J_M^(-1),16);   % J inverse Matrix
%delete(Timer);

limit = (det(J_M));
lim1 = 0; lim2 = pi/2;      % 나중에 solve로 바꾸기



%% ======================== continuous moving ========================

% FK : X0, -> q0 -> q1 -> J^(-1)

%q_new = simplify(J_M_inv*(X1-X0) + q_old);
assume(th3 > 0 & th3 < pi/2); % singularity

% Xdot, J_inv

dt = 0.01;
t = 0:dt:period-dt;


i=1;
MotorA=0;
MotorB=0;
MotorC=0;

wayPoints = [X1 X0];
timestamps = [0 period];
timevec = timestamps(1):dt:timestamps(end);

[X_new,Xd,Xdd] = cubicpolytraj(wayPoints,timestamps,timevec);


q0 = [pi/4; pi/4; pi/4]; %initial angle : 512
J0 = subs(J_M,[th1;th2;th3],q0);
J0_inv = J0^(-1);

for tt=1:dt:period+1
    if tt==1
        q_old=q0;
        J_inv = J0_inv;
        X_old = X0;
    else
        q_old=double(q_new);    %current param
        X_old = double(subs(MT0e(1:3, 4), [th1;th2;th3], q_old)); %FK, current param
    end
    q_new = J_inv*(X_new(:,i)-X_old) + q_old;
    
    theta1 = int64(rad2pos(q_new(1)));
    theta2 = int64(rad2pos(q_new(2)));
    theta3 = int64(rad2pos(q_new(3)));

    Motor1(i) = 2048 + theta1;
    Motor2(i) = 2048 + theta2;
    Motor3(i) = 2048 + theta3;

    q_gui(:,i) = double(q_new);

    i=i+1;
    %joint1.JointPosition = theta1(k);
%joint2.JointPosition = theta2(k);
end

% 
% for tt=1:dt:period+1 %while(q_new==q_old)
%     if(th3 == lim1 | th3 == lim2)
%         disp("Pseudo-Inverse");
%         break;
%     end
% 
%     if tt==1
%         q_old = q0;
%         J_inv = J0_inv;
%         X_new = X1;
%         X_old = X0;
% 
%     else
%         %disp("case 2");
%         q_old = double(q_new);
%         % X_old = X_new;
%         % X_new = double(subs(MT0e(1:3, 4), [th1;th2;th3], q_old));
%         % [th1;th2;th3] = subs(q_old);
%         % X_old = double(subs(MT0e(1:3, 4), [th1;th2;th3], q_old)); %FK
%         % X_new = X1;
% 
%         J = double(subs(J_M,[th1;th2;th3],q_old));
%         J_inv = J^(-1);
%     end
% 
%     q_new = J_inv*(X_new(:,i)-X_old) + q_old
%     q_old = q_new;
%     X_old = X_new(:,i);
% 
% 
%     % theta1 = int64(rad2pos(q(1)))
%     % theta2 = int64(rad2pos(q(2)))
%     % theta3 = int64(rad2pos(q(3)))
%     % 
%     % Motor1(i) = 2048 + theta1;
%     % Motor2(i) = 2048 + theta2;
%     % Motor3(i) = 2048 + theta3;
% 
%     i=i+1;
% 
%     % gui_dhparams = [0, 0, -d1, 0;
%     %     0, pi/2, L1, q_new(1);
%     %     0, 0, L2, -pi/2;
%     %     d2, -pi/2, 0, q_new(2);
%     %     0,-pi/2, 0, pi/2;
%     %     0,-pi/2, 0, pi/2;
%     %     L3, 0, 0, q_new(3);
%     %     L4, 0, 0, 0];
% 
% end


%% =============================== plot ===============================
% plot(timevec, X_new)
% hold all
% plot(timestamps, wayPoints, 'x')
% xlabel('t')
% ylabel('Positions')
% legend('X-positions','Y-positions','Z-positions')
% hold off
% plot(timevec, q_plot)
% hold all
% plot(timestamps, wayPoints, 'x')
% xlabel('t')
% ylabel('Positions')
% legend('X-positions','Y-positions')
% hold off
figure(1);
hold on
grid on
axis equal
%axis([-300 300 -300 300 0 400]);
view(3);
config = homeConfiguration(srcirc24_MDH);

i=1;
j=1;

for ttt=1:size(dhparams,1)
    T= double(subs(dhparams,[th1;th2;th3],q_gui(:,i)));
    j=1;
    setFixedTransform(joint(j),double(T(j,:)),'mdh');
    j=j+1;
end

pause(1);

for k=1:length(q_gui)
    config(1).JointPosition = q_gui(1,i);
    config(2).JointPosition = q_gui(2,i);
    config(3).JointPosition = q_gui(3,i);
    

    figure(2);
    show(srcirc24_MDH, config, 'PreservePlot', false);
    pause(0.05);
    i=i+1;
end


hold off


% 
% dhparams = [0, pi/2, 0, 0;
%             0, 0, L1,  th1;
%             0, 0, 0, -pi/2;
%             L3, -pi/2, L2, th2;
%             0,-pi/2, 0, pi/2;
%             -L4,-pi/2, 0, (pi/2)+th3;
%             L5, 0, 0, 0];
% 
% srcirc24_MDH = robotics.RigidBodyTree;
% 
% body1 = robotics.RigidBody('body1');
% jnt1 = robotics.Joint('jnt1','fixed');
% 
% setFixedTransform(jnt1,dhparams(1,:),'mdh');
% 
% figure;
% show(srcirc24_MDH);

%====================== Substitution equation ======================%

%q1 = subs(q1, L2*cos(th3)*sin(th2) - L3*sin(th3) + L4*cos(th3)*sin(th2), T);

% substitution sin cos
% q1 = subs(q1, sin(th1), s1);
% q1 = subs(q1, sin(th2), s2);
% q1 = subs(q1, sin(th3), s3);
% q1 = subs(q1, cos(th1), c1);
% q1 = subs(q1, cos(th2), c2);
% q1 = subs(q1, cos(th3), c3);
% 
% 
% q1 = subs(q1, L2*c3*s2 - L3*s3 + L4*c3*s2, T);
% 
% 
% syms K K2 C C2 C3 C4    % C : Constant value
% q1 = subs(q1, (L2*c1 + L4*c1 + L3*s1 + c3*s1*s2*L5 + L5*c1*s3)*c2*s3, K);
% q1 = subs(q1, L2*s1 - L3*c1 + L4*s1 + L5*s1*s3 - L5*c1*c3*s2, K2);
% q1 = subs(q1, (2^(1/2)*L2)/2 + (2^(1/2)*L3)/2 + (2^(1/2)*L4)/2 + (2^(1/2)*L5)/4, C);
% q1 = subs(q1, (2^(1/2)*L2)/2 - (2^(1/2)*L3)/2 + (2^(1/2)*L4)/2 - (2^(1/2)*L5)/4 + L5/2 - z1, C2);
% q1 = subs(q1, L1 + L5/2 + y1, C3);
% q1 = subs(q1, C + L5/2 - x1, C4);




%div = simplify(q1(2)/q1(1))

%simplify(atan2(q1(2), q1(1)))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function timer_callback(obj, event)
%disp(event.Data)
global Milli_Second;
global send_flag;
Milli_Second = Milli_Second + 0.01;
if Milli_Second >= 0.3
    Milli_Second = 0;
end

end

function pos = rad2pos(rad)
    pos = (((rad*180.0)/pi) * 4096.0) / 360.0;
end

function pos = deg2pos(deg)
    pos = (deg * 4096.0) / 360.0;
end


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