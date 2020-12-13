function [theta_,points] = IK_e2th(end_2)

x = end_2(1);
y = end_2(2);
z = end_2(3);

l1 = 0.038;
l2 = 0.128;
l3 = 0.024;
l4 = 0.124;
l5 = 0.141;

L3 = sqrt(l2*l2+l3*l3);
L4 = l4;

theta_3_offset = pi/2;%5/12*pi;

for psai = 0%-0.2-pi/2:0.1:0.4
    %tic
theta_1 = atan2(y,x);

Z = z-l5*sin(psai)-l1;
Y = sqrt(x*x+y*y)-l5*cos(psai);


theta_compen = atan2(l3,l2);

gamma1 = acos((L3*L3+L4*L4-Z*Z-Y*Y)/(2*L3*L4));
gamma2 = 2*pi-gamma1;
sigma = acos((L3*L3-L4*L4+Z*Z+Y*Y)/(2*L3*sqrt(Z*Z+Y*Y)));
%theta_3 = pi+theta_compen-gamma1-theta_3_offset;
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
theta_ = [theta_1; theta_2_1; theta_3;theta_4];

% check feasibility

if isreal(theta_) ~=true
    theta_ = real(theta_);
    disp('Tilt ee.');
    c3 = [0;0;l1];
    La = l4+norm([l2,l3]);
    Lb = l5;
    Lc = norm(c3 - end_2);
    
    theta_3_1 = atan((end_2(3)-c3(3))/(norm(end_2(1:2))));
    theta_3_2 = acos((La^2+Lc^2-Lb^2)/(2*La*Lc));
    theta_2_1 = pi/2 - (theta_3_1+theta_3_2+atan(l3/l2));            % theta3
    theta_3 = -pi/2+atan(l3/l2);                                     % theta4
    theta_4 = pi - acos((La^2+Lb^2-Lc^2)/(2*La*Lb));     % theta5
    
    theta_ = [theta_1; theta_2_1; theta_3;theta_4];
    
    if isreal(theta_) ~=true
        theta_ = real(theta_);
        disp('Infesible ee point.');
    end
    
end

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
%toc
points = [point1 point2 point3 point4 point5];

end