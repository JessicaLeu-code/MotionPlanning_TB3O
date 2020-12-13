%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RRT planning for TB3O 
% Robot model: TB3O
%
% Jessica Leu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all
clear all
%% Robot
% number of states
nstate = 7;
% number of inputs
dim = 7;
%%%%%%%%%%%%%
TB3O
%%%%%%%%%%%%%

%% Environment
view_area = robot.view_area;
% Obstacle
ran_=0;
obs{1}.shape = 'box';
obs{1}.c = [0 0.75 0]';
obs{1}.region = [0.05 0.45 0.3]';

obs{2}.shape = 'cylinder';
obs{2}.A = [1 0 ; 0 1];
obs{2}.c = [0.5 ; 0.5 ];
obs{2}.l = [[obs{2}.c; 0] [obs{2}.c; 0.6]]; % cylinder (straight line);% % for M200i
obs{2}.D = 0.15;
obs{2}.epsilon =  0.15;

obs{3}.shape = 'box';
obs{3}.c = [0.4 -0.7 0]';
obs{3}.region = [0.1 0.45 0.5]';

figure(1)
[plot_marker] = plot_obs_3D(obs,view_area);



%% Goal
% base
goalxyth = [0.9;0;-pi/3];
region_b = [0.05 0.05 pi/6]';
% ee
goalee = [1;0;0.25];
[Ax_next,end_next] = TB3O_IK(goalxyth,robot,goalee);
goal_th = [goalxyth(1:2); Ax_next]; %[0.4 0.3 1.5708 -0.6832 0.2532  -0.2055 -0.0477]';
goalxythxyz = [goalxyth;goalee];
region_g = [ region_b' 0.03 0.05 0.03]';

% plot goal
figure(1)
[end_all]=plot_MM5(1,Ax_next,goalxyth(1:2),robot,tb3,1,0,[],[]);




%% State constraints
region_s = [1  1  pi*1.2  pi   pi/2  pi/3    pi/1.5]';
sample_off = [0.3 0 0 0 0 0 0 ]';

%% System info

% initialize
start = [-0.4; 0.4];
base = [start; 0.1];
theta_b = pi*3/2;
ee_xyz0 = [-0.1;0.5;0.35];
% store info
sys_info.robot = robot;
sys_info.dim = dim;
sys_info.nstate = nstate;
sys_info.theta_b = theta_b;
[Ax_next,end_next] = TB3O_IK([base(1:2);theta_b],robot,ee_xyz0);
sys_info.x0 = [start; theta_b; Ax_next(2:end)];
sys_info.arm = Ax_next(2:end);
sys_info.base = base;
sys_info.goal_th = goal_th;
sys_info.ee_xyz0 = ee_xyz0;

xyRth = norm(start -goalxyth(1:2))/(pi*2);
sys_info.normalize = [1; 1; xyRth*ones(5,1)];

% plot initial 
figure(1)
[end_all]=plot_MM5(1,[theta_b; sys_info.arm],base(1:2),robot,tb3,1,0,[],[]);


%% solver RRT
tic
% single-thred
% self = RRT_tb3O(obs,sys_info,goalxythxyz,region_g,region_s,sample_off);
% self = self.RRTfind_all();
% multi-thred
self_ = {};
num_seed = 6;
routeL = 1000*ones(num_seed,1);
true_any = false;
while ~any(routeL<500)
    parfor i=1:num_seed 
    self_{i} = RRT_tb3O(obs,sys_info,goalxythxyz,region_g,region_s,sample_off);
    self_{i} = self_{i}.RRTfind_all();
        if self_{i}.fail ~=true
            routeL(i) = size(self_{i}.route,2);            
        end
    end    
end
%%
[path_length,id] = min(routeL);
self = self_{id};


Time_{2}= toc; 

%% Plot
figure(2)
[plot_marker] = plot_obs_3D(obs,view_area);
hold on
%plot all tree
plot(self.all_nodes(2,:),self.all_nodes(3,:),'ok');
plot3(self.all_nodes_XYZ(4,:),self.all_nodes_XYZ(5,:),self.all_nodes_XYZ(6,:),'ob');
path_length = size(self.route,2);
pathimplemented = self.route(1:2,:);
angleimplemented = [self.route(3,:); self.route(4:end,:) ];
[end_all]=plot_MM5(path_length,angleimplemented,pathimplemented,robot,tb3,1,0,[],[]);
%%
figure(3)
plot(pathimplemented(1,:))
hold on
plot(pathimplemented(2,:))
plot(angleimplemented(1,:))
legend('x_1','y_1','\theta_1')
figure(4)
plot(angleimplemented(2:end,:)')