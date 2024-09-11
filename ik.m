%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% expand() % 심볼릭 변수들로 된 식을 계산
% factor() % 식을 인수분해
% simple() % 식을 가장 간단한 형태로 나타냄.
% collect() % 내림차순 정리
% pretty() % 가독성 있게 수학식으로 표현
% findSymType() % 심볼릭 변수 표시
% subs() % 심볼릭 변수에 특정 값 넣어 계산
% ezplot() 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc, clear % 저장된 변수 또는 함수 제거
close all  % Clear Command Window

syms th1 th2 th3 h0 L1 L2 L3 L4 L5 px1 py1 pz1 px0 py0 pz0
pi=sym(pi);

%%%%%%%%%%% initial condition %%%%%%%%%%%
th10 = pi/4; th20 = pi/4; th30=pi/4;

XT01 = DHmodified(0, pi/2, 0, 0);
XT12 = DHmodified(0, 0, L1, th10);
XT23 = DHmodified(0, 0, 0, -pi/2);
XT34 = DHmodified(L3, -pi/2, L2, th20);
XT45 = DHmodified(0,-pi/2, 0, pi/2);
XT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th30);
XT6e = DHmodified(L5, 0, 0, 0);

XT0e = XT01 * XT12 * XT23 * XT34 * XT45 * XT56 * XT6e;
x1 = XT0e(1,4);
y1 = XT0e(2,4);
z1 = XT0e(3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MT01 = DHmodified(0, pi/2, 0, 0);
MT12 = DHmodified(0, 0, L1,  th1);
MT23 = DHmodified(0, 0, 0, -pi/2);
MT34 = DHmodified(L3, -pi/2, L2, th2);
MT45 = DHmodified(0,-pi/2, 0, pi/2);
MT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th3);
MT6e = DHmodified(L5, 0, 0, 0);


MT0e = simplify(MT01 * MT12 * MT23 * MT34 * MT45 * MT56 * MT6e)

px = simplify(MT0e(1,4));
py = simplify(MT0e(2,4));
pz = simplify(MT0e(3,4));

%theta1 = simplify(atan2(py,px));

%%%%%%%%%%%%%%%% JACOBIAN %%%%%%%%%%%%%%%%
%partial diff
dx1 = diff(px, th1);
dx2 = diff(px, th2);
dx3 = diff(px, th3);
dy1 = diff(py, th1);
dy2 = diff(py, th2);
dy3 = diff(py, th3);
dz1 = diff(pz, th1);
dz2 = diff(pz, th2);
dz3 = diff(pz, th3);

J=simplify([dx1 dx2 dx3; dy1 dy2 dy3; dz1 dz2 dz3]) %OK

J_inv = simplify(J^(-1));

limit = simplify(det(J));
lim1 = 0; lim2 = pi/2;

%%%%%%%%%%%% organizing %%%%%%%%%% value for Jacobian
q1 = [th1; th2; th3]; % goal angle
q0 = [pi/4; pi/4; pi/4]; % init angle(rad) : 512
X0 = [px0; py0; pz0]; % init coor
X1 = [px1; py1; pz1]; % target coor

% solve. q1 = J_inv * (X1-X0)+q0;

q1_R = simplify(J_inv * (X1-X0)+q0) % Right side


%th1_R = q1_R(1)
%eqn = th1 == q1_R(1)

%solve(eqn, th1)

assume(th3 > 0 | th3 < pi/2)



%[sol1, sol2, sol3] = solve([q1_R(1)==0, q1_R(2)==0, q1_R(3)==0])


%assume(x > 0) 
%sol = solve(eqn, th1);  %sol2 = vpasolve(eqn, th1, [0 pi])
%S = simplify(sol);




%q1 = simplify(J_inv * (X1-X0) + q0);



function [T] = DHmodified(a, alpha, d, th)

    [T] = [           cos(th)             -sin(th)             0               a;
           sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
           sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
                            0                    0             0               1];
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clc, clear % 저장된 변수 또는 함수 제거
% close all  % Clear Command Window
% 
% %% KINEMATICS
% syms th1 th2 th3 th4 th5 th6 L1 L2 L3 L4 L5
% pi=sym(pi);
% 
% % DHclassical(a, alpha, d, th)
% % DHmodified(a, alpha, d, th)
% 
% 
% 
% %% modified
% MT01 = DHmodified(0, pi/2, 0, 0);
% MT12 = DHmodified(0, 0, L1,  th1);
% MT23 = DHmodified(0, 0, 0, -pi/2);
% MT34 = DHmodified(L3, -pi/2, L2, th2);
% MT45 = DHmodified(0,-pi/2, 0, pi/2);
% MT56 = DHmodified(-L4,-pi/2, 0, (pi/2)+th3);
% MT67 = DHmodified(L5, 0, 0, 0);
% 
% MT02 = MT01 * MT12;
% MT03 = MT02 * MT23;
% MT04 = MT03 * MT34;
% MT05 = MT04 * MT45;
% MT06 = MT05 * MT56;
% MT07 = MT06 * MT67
% 
% %% check
% % if CT06 == MT06
% %     1
% % else
% %     0
% % end
% 
% 
% %% JACOBIAN
% 
% % modified
% MO1 = MT01(1:3, 4);
% MO2 = MT02(1:3, 4);
% MO3 = MT03(1:3, 4);
% MO4 = MT04(1:3, 4);
% MO5 = MT05(1:3, 4);
% MO6 = MT06(1:3, 4);
% MO7 = MT07(1:3, 4);
% 
% MZ1 = MT01(1:3, 3);
% MZ2 = MT02(1:3, 3);
% MZ3 = MT03(1:3, 3);
% MZ4 = MT04(1:3, 3);
% MZ5 = MT05(1:3, 3);
% MZ6 = MT06(1:3, 3);
% MZ7 = MT07(1:3, 3);
% 
% MO61 = MO6 - MO1;
% MO62 = MO6 - MO2;
% MO63 = MO6 - MO3;
% MO64 = MO6 - MO4;
% MO65 = MO6 - MO5;
% MO66 = MO6 - MO6;
% MO67 = MO6 - MO7;
% 
% MJ1 = [cross(MZ1, MO61); MZ1];
% MJ2 = [cross(MZ2, MO62); MZ2];
% MJ3 = [cross(MZ3, MO63); MZ3];
% MJ4 = [cross(MZ4, MO64); MZ4];
% MJ5 = [cross(MZ5, MO65); MZ5];
% MJ6 = [cross(MZ6, MO66); MZ6];
% MJ7 = [cross(MZ7, MO67); MZ7];
% 
% MJ = [MJ1, MJ2, MJ3, MJ4, MJ5, MJ6, MJ7]
% 
% %% function
% 
% function [T] = DHmodified(a, alpha, d, th)
%     [T] = [           cos(th)             -sin(th)             0               a;
%            sin(th)*cos(alpha)   cos(th)*cos(alpha)   -sin(alpha)   -d*sin(alpha);
%            sin(th)*sin(alpha)   cos(th)*sin(alpha)    cos(alpha)    d*cos(alpha);
%                            0                    0             0               1];
% end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %pkg load control
% clear;							% 변수 초기화
% dist = [0.6, 0.5, 0];			% 목표 거리 [m], [m], [rad]
% max_step_x = 0.1;					% x방향 최대 보폭 [m]
% max_step_y = 0.05;				% y방향 최대 보폭 [m]
% max_step_w = 0.2;					% 회전 최대 보폭 [rad]
% period = 0.3;					% 보행 주기 [s]
% foot_y = 0.06;					% 발의 y 위치 [m]
% step_num_x = fix(abs(dist(1))/max_step_x);
% step_num_y = fix(abs(dist(2))/max_step_y);
% step_num_w = fix(abs(dist(3))/max_step_w);
% step_num = max([step_num_x, step_num_y, step_num_w]);
% if (step_num_x == step_num)
% 	step_x = sign(dist(1))*min(abs(dist(1)), max_step_x);
% 	step_y = dist(2)/abs(dist(1))*max_step_x;
% 	step_w = dist(3)/abs(dist(1))*max_step_x;
% elseif (step_num_y == step_num)
% 	step_x = dist(1)/abs(dist(2))*max_step_y;
% 	step_y = sign(dist(2))*min(abs(dist(2)), max_step_y);
% 	step_w = dist(3)/abs(dist(2))*max_step_y;
% elseif (step_num_w == step_num)
% 	step_x = dist(1)/abs(dist(3))*max_step_w;
% 	step_y = dist(2)/abs(dist(3))*max_step_w;
% 	step_w = sign(dist(3))*min(abs(dist(3)), max_step_w);
% end
% 
% foot(1,:) = [0 0 0];
% foot(2,:) = [period 0 foot_y];
% rot = 0;
% for i = 2:step_num+1
% 	shift_y = foot_y * (rem(i,2)*2-1)*2;
% 	foot_rd = [step_x, step_y+shift_y];
% 	foot_fd = [foot_rd(1)*cos(rot)-foot_rd(2)*sin(rot), foot_rd(1)*sin(rot)+foot_rd(2)*cos(rot)];
% 	rot = rot + step_w
% 	foot(i+1,:) = foot(i,:)+[period, foot_fd(1), foot_fd(2)];
% end
% foot_rd = [dist(1)-step_x*step_num, dist(2)-step_y*step_num-shift_y];
% foot_fd = [foot_rd(1)*cos(rot)-foot_rd(2)*sin(rot), foot_rd(1)*sin(rot)+foot_rd(2)*cos(rot)];
% foot(step_num+3,:) = foot(step_num+2,:) + [period, foot_fd(1), foot_fd(2)];
% rot = dist(3)
% foot_rd = [0,shift_y/2];
% foot_fd = [foot_rd(1)*cos(rot)-foot_rd(2)*sin(rot), foot_rd(1)*sin(rot)+foot_rd(2)*cos(rot)];
% foot(step_num+4,:) = foot(step_num+3,:) + [period, foot_fd(1), foot_fd(2)];
% foot(step_num+5,:) = [100, 0, 0];
% 
% %Design of an optimal controller for a discrete-time system subject to previewable demand 참고
% 
% % Matrix A, B, C, and D from cart table model
% forward_period = 1.0;						% 예측 제어 시간 (s)
% calculate_period = 4.0;						% 보행 패턴 생성 기간 (s)
% dt = 0.01;								% 샘플링 시간 (s)
% zh = 0.27;								% 무게 중심 위치 (m)
% g  = 9.8;								% 중력 가속도 (m/s^2)
% A = [0 1 0; 0 0 1; 0 0 0];					% 식(5.8) 상태행렬
% B = [0; 0; 1];							% 입력행렬
% C = [1 0 -zh/g];							% 식(5.9) 출력행렬
% D = 0;								% 피드백 행렬
% 
% % Convert continuous system to discrete system
% sys = ss(A, B, C, D);
% sys_d = c2d(sys, dt);						% 이산화
% % [A_d, B_d, C_d, D_d] = sys2ss(sys_d);
% [A_d, B_d, C_d, D_d] = ssdata(sys_d);		% 식(5.10)
% 
% % A, B, C matrix for LQI(Linear Quadratic Integral) control
% E_d = [dt; 1; 0];							% 식(5.11)
% Zero = [0; 0; 0];							% 식(5.12),(5.13)
% Phai = [1 -C_d*A_d; Zero A_d];
% G = [-C_d*B_d; B_d];
% GR = [1; Zero];
% Gd = [-C_d*E_d; E_d];
% % Quadratic Cost Function(LQR제어 응용)
% % Q matrix
% Q = zeros(4);
% Q(1) = 10^8;						% ★★★ 최적 제어의 가중치 계수, 클수록 ZMP가 이상에 가까워짐
% H = 1;							% 마찬가지로 가중치 계수, 클수록: 입력이 줄어듦
% 
% % P = ricatti equation
% P = dare(Phai, G, Q, H);					% 이산 시간계에서의 리카치 방정식의 해
% F = -(H+G'*P*G)^(-1)*G'*P*Phai;			% ★★★ 피드백 게인 식(2.13)
% 
% x = [0; 0; 0];
% y = [0; 0; 0];
% xp = x;
% yp = x;
% 
% t = 0:dt:calculate_period;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% i = 1;								% 목표 ZMP 형상 생성
% n = 1;							% 발의 착지 시간 및 위치 foot → 목표 ZMP 패턴 prefx, prefy
% for tt = 0:dt:calculate_period+forward_period+1
% 	if (abs(tt - foot(n,1))<(dt/2))
% 		prefx(i) = foot(n,2);
% 		prefy(i) = foot(n,3);
% 		n = n + 1;
% 	else
% 		prefx(i) = prefx(i-1);
% 		prefy(i) = prefy(i-1);
%     end
% 	i = i + 1;
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% i = 0;
% ux = 0; uy = 0;
% 
% xi = (eye(4)-G*(H+G'*P*G)^(-1)*G'*P)*Phai;
% 
% for tt = t
% 	i = i + 1;
% %	fd = (H+G'*P*G)^(-1)*G'*(xi')^1*P*Gd;	% 외란 제거용 미사용
% 	px = C_d*x;						% 실제 ZMP 위치
% 	py = C_d*y;
% 	ex = prefx(i) - px;					% 목표 ZMP 위치와 실제와의 차이
% 	ey = prefy(i) - py;
% 	X = [ex; x - xp];					% 식(2.5),(2,6)
% 	Y = [ey; y - yp];
% 	xp = x;						% 차분을 계산하기 위해 이전 값 저장
% 	yp = y;
% 	dux = F * X;						% 식(2.8)
% 	j = 0;
% 	for ttt = tt : dt : (tt + forward_period)
% 		j = j + 1;
% 		if (prefx(i+j) - prefx(i+j-1)) ~= 0
% 			f  = -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;		% ★★★ 식(2.14)
% 			dux = dux + f * (prefx(i+j) - prefx(i+j-1));		% 식(2.12)
%         end
%     end
% 	ux = ux + dux;
% 	duy = F * Y;									% y 방향 예측 제어
% 	j = 0;
% 	for ttt = tt : dt : (tt + forward_period)
% 		j = j + 1;
% 		if (prefy(i+j) - prefy(i+j-1)) ~= 0
% 			f  = -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;		% ★★★ 식(2.14)
% 			duy = duy + f * (prefy(i+j) - prefy(i+j-1));		% 식(2.12)
%         end
%     end
% 	uy = uy + duy;
% 
% 	dx = 0;									% 외란 (여기서는 사용 안 함)
% 	dy = 0;
% 	x = A_d * x + B_d * ux + E_d * dx * dt;		% 무게 중심 위치 계산(COM_pos)
% 	y = A_d * y + B_d * uy + E_d * dy * dt;
% 	x0(i) = x(1);							% 무게 중심 위치 그리기용(COM_pos)
% 	y0(i) = y(1);
% 	x1(i) = prefx(i);						% 목표 ZMP 위치 그리기용(ZMP_goal)
% 	y1(i) = prefy(i);
% 	x2(i) = px;							% 실제 ZMP 위치 그리기용(ZMP_real)
% 	y2(i) = py;
% end
% subplot(2,1,1);							% 그래프 그리기
% plot(x0, y0, x1, y1, x2, y2);
% legend('COM pos ', 'ZMP goal',  'ZMP real');
% subplot(2,1,2);
% plot(t, x0, t, x1, t, x2, t, y0, t, y1, t, y2);
% legend('COM pos x', 'ZMP goal x', 'ZMP real x', 'COM pos y', 'ZMP goal y', 'ZMP real y');