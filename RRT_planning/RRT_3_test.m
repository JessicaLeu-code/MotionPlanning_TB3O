%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RRT planning for TB3 
% Robot model: TB3 ( mobile platform)
%
% Jessica Leu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all

% robot
%%%%%%%%%%%%%
TB3O
%%%%%%%%%%%%%

%% Environment
view_area = robot.view_area;

% Obstacle
obs{1}.shape = 'box';
obs{1}.c = [0 0.75 0]';
obs{1}.region = [0.1 0.45 0.3]';

obs{2}.shape = 'box';
obs{2}.c = [0.8 0 0]';
obs{2}.region = [0.1 0.45 0.5]';

obs{3}.shape = 'box';
obs{3}.c = [0.1 -0.6 0]';
obs{3}.region = [0.45 0.1 0.5]';

figure(1)
[plot_marker] = plot_obs_3D(obs,view_area);
hold on
% initialize
start = [-0.4; 0.4];
base = [start; 0.1];
theta_b = pi*3/2;
% Goal
goalxyth = [1.2;0;-pi/3];
goal_th = goalxyth;
region_g = [0.05 0.05 pi/6]';
% Robot
% number of states
nstate = 3;
% number of inputs
dim = 3;

%% State constraints
region_s = [1  1  pi*1.2]';
sample_off = [0.5 0.2 0]';

%% System info
sys_info.robot = robot;
sys_info.dim = dim;
sys_info.nstate = nstate;
sys_info.theta_b = theta_b;
sys_info.x0 = [start; theta_b];
sys_info.arm = [0;0;0;0];
sys_info.base = base;
sys_info.goal_th = goal_th;

% plot initial 
figure(1)
[end_all]=plot_MM5(1,[theta_b; sys_info.arm],base(1:2),robot,tb3,1,0,[],[]);

%% solver RRT
% multi-thread
tic
self_ = {};
num_seed = 6;
routeL = 1000*ones(num_seed,1);
true_any = false;
while ~any(routeL<100)
    parfor i=1:num_seed 
    self_{i} = RRT_tb3O(obs,sys_info,goalxyth,region_g,region_s,sample_off,'base');
    self_{i} = self_{i}.RRTfind_b();
        if self_{i}.fail ~=true
            routeL(i) = size(self_{i}.route,2);            
        end
    end    
end
[path_length,id] = min(routeL);
self = self_{id};
% single-thread
% tic
% self = RRT_tb3O(obs,sys_info,goalxyth,region_g,region_s,sample_off,'base');
% self = self.RRTfind_b();
Time_{2}= toc 
        
%% ILQR
% rearrange heading angle
for i = 1:size(self.route,2)-1
    self_a=self.route(3,i+1);
    up_a = self.route(3,i+1)+2*pi;
    down_a =self.route(3,i+1)-2*pi;
    d_s = norm(self.route(3,i)-self_a);
    d_u = norm(self.route(3,i)-up_a);
    d_d = norm(self.route(3,i)-down_a);
    if d_s<d_u
        if d_d<d_s
            self.route(3,i+1) = self.route(3,i+1)-2*pi;            
        end
    else
        if d_d<d_u
            self.route(3,i+1) = self.route(3,i+1)-2*pi;    
        else
            self.route(3,i+1) = self.route(3,i+1)+2*pi;
        end 
    end
end
% ILQR setup
var.N = size(self.route,2);
var.H = var.N-1;
var.basethT = goalxyth(3);
states_out = [self.route; zeros(2,size(self.route,2))];
[x_out, u] = base_ILQR(states_out,var); 

%% Plot
figure(2)
[plot_marker] = plot_obs_3D(obs,view_area);
hold on
path_length = size(x_out,2);
pathimplemented = x_out(1:2,:);
angleimplemented = [x_out(3,:); kron(ones(1,path_length),sys_info.arm)];

plot(goalxyth(1),goalxyth(2),'*r')
all = plot(self.all_nodes(2,:),self.all_nodes(3,:),'ok');
plot(x_out(1,:),x_out(2,:),'-b*');
[end_all]=plot_MM5(path_length,angleimplemented,pathimplemented,robot,tb3,1,0,[],[]);
%%
figure(3)
rrt = plot(self.route(1,:)','r--');
hold on
plot(self.route(2:3,:)','r--');
ri(1)= plot(pathimplemented(1,:));
ri(2)=plot(pathimplemented(2,:));
ri(3)=plot(angleimplemented(1,:));
legend([rrt ri(1:3)],'RRT','x(k)','y(k)','\theta(k)')
xlabel('Time steps')
ylabel('x [m]/ y [m] / Angle [rad]')
