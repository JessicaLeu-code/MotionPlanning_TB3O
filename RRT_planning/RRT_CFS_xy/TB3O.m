%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setups for TB3O 
% Robot model: TB3O
%
% Jessica Leu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TB3 model

tb3.base = [0.065 0.065 -0.195 -0.195;
                0.13  -0.13 -0.13  0.13];
            
tb3.wC = [0    0;
         0.13  -0.13  ;
         0.05 0.05];   % center of circle 
R = 0.0215 ;    % Radius of circle 
teta=0:0.5:2*pi ;
for i = 1:2
tb3.wx(i,:) = tb3.wC(1,i)+R*cos(teta);
tb3.wy(i,:) = tb3.wC(2,i)*ones(1,length(teta)) ;
tb3.wz(i,:) = tb3.wC(3,i)+R*sin(teta) ;
end

%% robot
robot.cap={};
robot.cap{1}.p=[0.05 -0.25;0 0;-0.05 -0.05];
robot.cap{1}.r=0.15;

robot.cap{2}.p=[0 0;0 0;0 0.038];
robot.cap{2}.r=0.02;

robot.cap{3}.p=[ 0  0.024;0 -0.128;0 0];
robot.cap{3}.r=0.02;

robot.cap{4}.p=[0 0.124;0 0;0 0];
robot.cap{4}.r=0.02;

%         robot.cap{5}.p=[0 0.12;0 0;0 0];
%         robot.cap{5}.r=0.025; 

robot.cap{5}.p=[0 0.141;0 0;0 0];
robot.cap{5}.r=0.03; 



robot.r = 0.0215;
robot.d = 0.13;

robot.T = [0   0 -0.15    0   0.024 0.124;
           0   0   0      0   -0.128   0 ;
           0   0   0.0  0.038     0    0];
       
% base corner info
base = [0.065 0.065 -0.195 -0.195;
        0.13  -0.13 -0.13   0.13;
        0      0     0      0];
robot.cap{1}.p = [robot.cap{1}.p zeros(3,2) base];

% include tb3 (base)
robot.tb3 = tb3;
       
% view
robot.view_area = [-1 2 -1 1.5 0 0.6];



%% ILQR
var.H = 20;
var.nstep = var.H+1;
var.r = 0.0215;
%var.Vref = Vref;
var.nstate = 5;
var.dt = 0.2;
var.N = 25;
var.nx = 5;
var.nu = 2;
var.Thres=1e-4;
var.lineSearchThres=1e-4;
%dim = 2;
var.basethT = 0;