close all
clear all

l1 = 0.038;
l2 = 0.128;
l3 = 0.024;
l4 = 0.124;
l5 = 0.141;

L3 = sqrt(l2*l2+l3*l3);
L4 = l4;

theta_3_offset = 5/12*pi;

theta_ = [];

for psai = 0%-0.2-pi/2:0.1:0.4

tic
x = 0.15;
y = 0.1;
z = -0.05;


theta_1 = atan2(y,x);

Z = z-l5*sin(psai)-l1;
Y = sqrt(x*x+y*y)-l5*cos(psai);


theta_compen = atan2(l3,l2);

gamma1 = acos((L3*L3+L4*L4-Z*Z-Y*Y)/(2*L3*L4));
gamma2 = 2*pi-gamma1;
sigma = acos((L3*L3-L4*L4+Z*Z+Y*Y)/(2*L3*sqrt(Z*Z+Y*Y)));
theta_3 = pi+theta_compen-gamma1-theta_3_offset;
% first try gamma1
% TODO gamma2
% TODO test availability

A = L3-L4*cos(gamma1);
B = L4*sin(gamma1);

alpha = atan2(Y,Z)-sigma;
theta_2_1 = alpha-theta_compen;

% TODO bug fixed 
%alpha = 2*(atan((A+sqrt(A*A+B*B-Z*Z))/(B+Z)))
%theta_2_1 = alpha-theta_compen;
%theta_2_2 = 2*(atan((A-sqrt(A*A+B*B-Z*Z))/(B+Z)));

beta = gamma1-theta_2_1-pi/2;



theta_4 = pi/2-theta_3_offset-psai-theta_2_1-theta_3;
% theta_1 = 0
% theta_2_1 = 0
% theta_3 = 0
% theta_4 = 0
% alpha = theta_2_1+theta_compen

point1 = [0 0 0 0]';

point2 = [0 0 l1 0]';

point3 = [L3*cos(theta_1)*sin(theta_2_1+theta_compen) L3*sin(theta_1)*sin(theta_2_1+theta_compen) l1+L3*cos(theta_2_1+theta_compen) 0]';

point4 = [(L3*cos(theta_1)*sin(theta_2_1+theta_compen)+L4*cos(theta_1)*sin(theta_2_1+theta_3_offset+theta_3)) 
            L3*sin(theta_1)*sin(theta_2_1+theta_compen)+L4*sin(theta_1)*sin(theta_2_1+theta_3_offset+theta_3)
            l1+L3*cos(theta_2_1+theta_compen)+L4*cos(theta_2_1+theta_3_offset+theta_3)
            0];
point5 = [(L3*cos(theta_1)*sin(theta_2_1+theta_compen)+L4*cos(theta_1)*sin(theta_2_1+theta_3_offset+theta_3)+l5*cos(theta_1)*sin(theta_2_1+theta_3_offset+theta_3+theta_4))
            L3*sin(theta_1)*sin(theta_2_1+theta_compen)+L4*sin(theta_1)*sin(theta_2_1+theta_3_offset+theta_3)+l5*sin(theta_1)*sin(theta_2_1+theta_3_offset+theta_3+theta_4)
            l1+L3*cos(theta_2_1+theta_compen)+L4*cos(theta_2_1+theta_3_offset+theta_3)+l5*cos(theta_2_1+theta_3_offset+theta_3+theta_4)
            0];   
toc
points = [point1 point2 point3 point4 point5];
xpoints = points(1,:);
ypoints = points(2,:);
zpoints = points(3,:);
axis equal 
for i = 1:4
    line(xpoints(i:i+1),ypoints(i:i+1),zpoints(i:i+1))
    hold on
end
grid on
hold off
view([1,-1,1])
xlabel('x')
ylabel('y')
zlabel('z')
pause(0.1)
theta_ = [theta_  [theta_1; theta_2_1; theta_3;theta_4;]];
end
