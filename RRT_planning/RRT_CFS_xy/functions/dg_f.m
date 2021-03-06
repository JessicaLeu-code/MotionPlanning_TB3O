function dg = dg_f(A,c,x)
%DG_F
%    DG = DG_F(A1_1,A1_2,A2_1,A2_2,C1,C2,X1,X2)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    27-Mar-2020 17:16:32

A1_1 = A(1,1);
A1_2 = A(1,2);
A2_1 = A(2,1);
A2_2 = A(2,2);
c1 = c(1);
c2 = c(2);
x1 = x(1);
x2 = x(2);


t2 = conj(c1);
t3 = conj(c2);
t4 = conj(x1);
t5 = conj(x2);
t6 = -x1;
t7 = -x2;
t8 = -t4;
t9 = -t5;
t10 = c1+t6;
t11 = c2+t7;
t12 = t2+t8;
t13 = t3+t9;
dg = [A1_1.*t10+A1_1.*t12+A1_2.*t11+A2_1.*t13;A1_2.*t12+A2_1.*t10+A2_2.*t11+A2_2.*t13];
