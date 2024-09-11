function x=Jacobi(A, b, x0, imax, epsilon)
%입력 : A, b, imax, x0
%출력 : x
% A : 계수행렬,  b : 상수행렬
% x0 : 초기 가정값 - 책에서의 x_old
% x : 해 - 책에서의 x_new 
% 계수를 변환한다.
n=length(b);
Ahat=A;
bhat=b;
x=x0;
for i =1: n
		Ahat(i,1 : n) = A(i,1 : n)/A(i,i);
		bhat(i) = b(i)/A(i,i);
		Ahat(i,i)= 0.;
    end
% 반복 대입하여 계산한다 
for iter = 1 : imax 
	for i =1 : n
		x(i)=bhat(i)-Ahat(i,1 : n)*x0(1 : n);  
    end
% 수렴판단을 한다." 
	ressum=0;
	for i =1 : n
        sum=	x(i) - ( bhat(i) - Ahat(i,1 : n)*x(1 : n) ) ;  
		res(i)= abs(sum);
		ressum=ressum+res(i);
    end
	resavg=ressum/n;
	if (resavg < epsilon)
        iter
        resavg
  		return
    end
		x0=x;
end
        iter
        resavg

% 수렴하지 않는다.
disp('수렴하지 않는다') 
