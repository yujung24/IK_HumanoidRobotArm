function x=Jacobi(A, b, x0, imax, epsilon)
%�Է� : A, b, imax, x0
%��� : x
% A : ������,  b : ������
% x0 : �ʱ� ������ - å������ x_old
% x : �� - å������ x_new 
% ����� ��ȯ�Ѵ�.
n=length(b);
Ahat=A;
bhat=b;
x=x0;
for i =1: n
		Ahat(i,1 : n) = A(i,1 : n)/A(i,i);
		bhat(i) = b(i)/A(i,i);
		Ahat(i,i)= 0.;
    end
% �ݺ� �����Ͽ� ����Ѵ� 
for iter = 1 : imax 
	for i =1 : n
		x(i)=bhat(i)-Ahat(i,1 : n)*x0(1 : n);  
    end
% �����Ǵ��� �Ѵ�." 
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

% �������� �ʴ´�.
disp('�������� �ʴ´�') 
