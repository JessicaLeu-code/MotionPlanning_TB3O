%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RRT class file for TB3O 
% Robot model: TB3O
%
% Jessica Leu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef RRT_tb3O
   properties
       % setup parameters
       obs cell
       sys_info struct
       goal {mustBeNumeric}
       region_g {mustBeNumeric}
       region_s {mustBeNumeric}
       sample_off {mustBeNumeric}
       dt = 0.2;
       
       % intermediate states
       parent {mustBeNumeric}
       newNode {mustBeNumeric}
       newNodeXYZ {mustBeNumeric}
       newVW {mustBeNumeric}
       check_points {mustBeNumeric}
       isGoalReached logical
       v {mustBeNumeric}
       w {mustBeNumeric}
       
       % outputs
       all_nodes {mustBeNumeric}                   % all nodes
       all_nodes_XYZ {mustBeNumeric}               % all nodes
       all_VW {mustBeNumeric}               % all base input
       route {mustBeNumeric}
       fail = 0;   
           
       % sover settings
       MAX_ITER = 1000;  %parallel 200
       bi = 0.6;
       prevLalpha = 0;
       SOLVER_info = 'all'                 % base-only
       margen = 0.02;
              
       % calculation statistics
       node_num = 1;
              
       
   end
   
   
   methods
       % _init_
       function self = RRT_tb3O(val,val2,val3,val4,val5,val6,varargin)  

            % get problem info
            self.obs = val;
            self.sys_info = val2;                
            self.goal = val3;
            self.region_g = val4;
            self.region_s = val5;
            self.sample_off = val6;
            if ~isempty(varargin)
                self.SOLVER_info = varargin{1};                    
            end 
                               
       end
       
       
       % main function      
       function self = RRTfind_b(self)
           % initialize tree
           self.all_nodes = [1; self.sys_info.x0];
           self.newNode = self.sys_info.x0;
           self.newNodeXYZ  = self.sys_info.x0;
           self = self.goal_reached();
           % grow tree          
           while self.isGoalReached ~=true
               self = self.getNode_b();
               self = self.addNode();               
               self = self.goal_reached();
               
           end 
           
           self.route = self.newNode;
           while self.parent ~= 1
               self.route = [ self.all_nodes(2:end,self.parent) self.route];
               self.parent = self.all_nodes(1,self.parent);
           end
       end
       
       function self = RRTfind_all(self)
           % initialize tree
           self.all_nodes = [1; self.sys_info.x0];
           self.newNode = self.sys_info.x0;
           self.newNodeXYZ  = [self.sys_info.x0(1:3);self.sys_info.ee_xyz0];
           self  = self.goal_reached();
           % grow tree          
           while self.isGoalReached ~=true
               self = self.getNode_all();
               self = self.addNode();               
               self = self.goal_reached();
               
           end 
           
           self.route = self.newNode;
           while self.parent ~= 1
               self.route = [ self.all_nodes(2:end,self.parent) self.route];
               self.parent = self.all_nodes(1,self.parent);
           end
       end
       
       
       
       %%%%%%%%%%%%%%%% solver functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       function  self = getNode_all(self)
           self = self.getRandNode_all();
           [isFeasible, self]= self.feasible();
           while isFeasible ~=true
               self = self.getRandNode_all();
               [isFeasible, self]= self.feasible();
               not_fe =1;
           end               
       end 
       
       function  self = getNode_b(self)
           self = self.getRandNode_VW();
           [isFeasible, self]= self.feasible();
           while isFeasible ~=true
               self = self.getRandNode_VW();
               [isFeasible, self]= self.feasible();
               not_fe =1;
           end               
       end                 
       
       %%%%%%%%%%%%%%%% supporting functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       function self = getRandNode_all(self)
            samp=[];
            pp = rand;
           if pp<self.bi
               % specified sample range 
               sampleNode = (rand(self.sys_info.nstate,1)-0.5).*self.region_s*2+self.sample_off;
               % full joint space sampling
               % sampleNode = [self.sys_info.theta_b; (rand(self.sys_info.nstate,1)-0.5).*self.region_s*2];
           else
               sampleNode = self.sys_info.goal_th;
           end                  
           
           ori_node = sampleNode;
           dis  = norm((self.all_nodes(2:end,1)-sampleNode).*self.sys_info.normalize);
           % mirror (first arm joint)
           if sampleNode(4,1)>0
               mirror = [sampleNode(1:3,1); sampleNode(4,1)-pi*2; sampleNode(5:end,1)];
           else
               mirror = [sampleNode(1:3,1); sampleNode(4,1)+pi*2; sampleNode(5:end,1)];
           end 
           dis2  = norm((self.all_nodes(2:end,1)-mirror).*self.sys_info.normalize);
           if dis2<dis
               sampleNode = mirror;
               dis = dis2;
           end
               
           self.parent = 1; 
           for i = 2:self.node_num
               dis_  = norm((self.all_nodes(2:end,i)-sampleNode).*self.sys_info.normalize);
               dis2_  = norm((self.all_nodes(2:end,1)-mirror).*self.sys_info.normalize);
               if dis2_<dis_
                   sampleNode = mirror;
                   dis_ = dis2_;
               else
                   sampleNode = ori_node;                   
               end
               if dis_<dis
                   self.parent = i;
                   dis = dis_;
               end               
           end
           th_k = self.all_nodes(4,self.parent);
           self = self.get_VWold(sampleNode(1:3));
           % Next node
           % base
           self.newNode(1:3) = self.all_nodes(2:4,self.parent)...
                               +self.dt*[cos(th_k+self.w*self.dt*0.5)*self.v;...
                                         sin(th_k+self.w*self.dt*0.5)*self.v;...
                                                                      self.w];
           % arm
           self.newNode(4:7) = self.all_nodes(5:end,self.parent)+(sampleNode(4:end)-self.all_nodes(5:end,self.parent))*0.2/norm(self.all_nodes(5:end,self.parent)-sampleNode(4:end));
           self.newNode(3) = mod(self.newNode(3),pi*2);
           self.newVW = [self.v;self.w];
           
           % temp
           self.sys_info.arm = self.newNode(4:end);
       end
       
       function self = getRandNode_VW(self)
            samp=[];
            pp = rand;
           if pp<self.bi
               % specified sample range 
               sampleNode = (rand(self.sys_info.nstate,1)-0.5).*self.region_s*2+self.sample_off;
           else
               sampleNode = self.sys_info.goal_th;
           end
                     
           dis  = norm(self.all_nodes(2:3,1)-sampleNode(1:2));
           dis_th = norm(self.all_nodes(end,1)-sampleNode(end));
           self.parent = 1; 
           for i = 2:self.node_num
               dis_  = norm(self.all_nodes(2:3,i)-sampleNode(1:2));
               %%%
               if dis_<dis
                   self.parent = i;
                   dis = dis_;
                   dis_th = norm(self.all_nodes(end,i)-sampleNode(end));
               elseif norm(dis_-dis)<0.001
                   dis_th_ = norm(self.all_nodes(end,i)-sampleNode(end));
                   if dis_th_<dis_th
                       self.parent = i;
                       dis = dis_;
                       dis_th = dis_th_;
                   end
               end
               
           end
           % Next node
           % base
           th_k = self.all_nodes(4,self.parent);
           self = self.get_VWold(sampleNode);
           self.newNode = self.all_nodes(2:end,self.parent)...
               +self.dt*[cos(th_k+self.w*self.dt*0.5)*self.v;...
                         sin(th_k+self.w*self.dt*0.5)*self.v;...
                                                      self.w];
               
           self.newNode(3) = mod(self.newNode(3),pi*2);
           self.newVW = [self.v;self.w];

       end      
       
       %%%%%%%%%%%%%%%%%%% supplementary functions %%%%%%%%%%%%%%%%%%%%%%%%
       function self = get_VWold(self,sampleNode)
                       xy_diff = sampleNode(1:2) - self.all_nodes(2:3,self.parent);           
            th_k1 = atan(xy_diff(2)/xy_diff(1));
            if xy_diff(1)<0
                th_k1 = th_k1+pi;
            else
                if th_k1<0
                    th_k1 = th_k1+2*pi;
                end
            end
            
           th_k = self.all_nodes(4,self.parent);
           if (th_k1-th_k)<0
               mirror = pi*2+th_k1;
           else
               mirror = th_k1-pi*2;
           end
           [dis_, id] = min([norm(th_k-th_k1),norm(th_k-mirror)]);
           if id == 2
               th_k1 = mirror;                       
           end
           
           v_ = min([0.25,norm(sampleNode(1:2)-self.all_nodes(2:3,self.parent))]);
           v_ = max([v_,0.01]);
           
           if v_>0.02
               w_ = th_k1-th_k;               
           else
               
               v_ = 0;
               goal_th = self.sys_info.goal_th(3);
               if (goal_th-th_k)<0
                   mirror = pi*2+goal_th;
               else
                   mirror = goal_th-pi*2;
               end
               [dis_, idth] = min([norm(th_k-goal_th),norm(th_k-mirror)]);
               if idth == 2
                   goal_th = mirror;                       
               end
               w_ = goal_th - th_k ;
           end
           self.v = v_;
           self.w = w_;
       end
       
       
       function [isFeasible, self]= feasible(self)
           %self = self.get_ee();   % arm only 
           self = self.get_checkpoint();          
           isFeasible = true;           
           for j = 1:size(self.obs,2)               
               switch self.obs{j}.shape
                   case 'box'                       
                       for i = 1:size(self.check_points,2)
                           if (self.obs{j}.c-(self.obs{j}.region+self.margen))<self.check_points(:,i)
                               if self.check_points(:,i)<(self.obs{j}.c+(self.obs{j}.region+self.margen))
                                   isFeasible = false;
                                   break;
                               else
                                   isFeasible = true;
                               end
                           else
                               isFeasible = true;
                           end
                       end                       
                   case 'cylinder'
                       for i = 1:size(self.check_points,2)
                           if norm(self.check_points(1:2,i) - self.obs{j}.l(1:2,1))<self.obs{j}.epsilon && self.check_points(3,i)<self.obs{j}.l(3,2)
                              isFeasible = false;
                              break;
                           else
                              isFeasible = true;
                           end   
                       end                       
                   case 'plate'
                       for i = 1:size(self.check_points,2)
                           if (self.obs{j}.c-self.obs{j}.region)<self.check_points(:,i)
                               if self.check_points(:,i)<(self.obs{j}.c+self.obs{j}.region)
                                   isFeasible = false;
                                   break;
                               else
                                   isFeasible = true;
                               end
                           else
                               isFeasible = true;
                           end
                       end                       
               end % switch
               if ~isFeasible
                   break
               end
           end
           self.check_points =[];
       end
       
       function self = addNode(self)           
           self.all_nodes = [self.all_nodes [self.parent; self.newNode]];
           self.all_nodes_XYZ = [self.all_nodes_XYZ self.newNodeXYZ];
           self.all_VW = [self.all_VW self.newVW];
           self.node_num = self.node_num + 1;
           
       end
             
       function self = goal_reached(self)
           %self = self.get_ee();
           self.isGoalReached = false;           
           if (self.goal-self.region_g)<self.newNodeXYZ 
               if self.newNodeXYZ<(self.goal+self.region_g)
                   self.isGoalReached = true;
               end
           end
           mirror = self.goal;          
           if self.goal(3)>0
               mirror(3) = mirror(3)-pi*2;
           else
               mirror(3) = mirror(3)+pi*2;
           end 
           if (mirror-self.region_g)<self.newNodeXYZ 
               if self.newNodeXYZ<(mirror+self.region_g)
                   self.isGoalReached = true;
               end
           end
           
           if self.node_num>self. MAX_ITER
               disp('Failed to find path.')
               self.fail = true;
               self.isGoalReached = true;
           end
                      
       end
                    
       function self = get_checkpoint(self)
            RoCap = self.sys_info.robot.cap;
            T = self.sys_info.robot.T;
            theta = [self.newNode(3);self.sys_info.arm];
            base = [self.newNode(1:2); 0.1];
            tb3 = self.sys_info.robot.tb3;

            nlink=size(theta,1);
            M=cell(1,nlink+1); M{1}=eye(4);

            for i=2:nlink+1
                RoCap{i-1}.p(:,3) = [0;0;0];
                RoCap{i-1}.p(:,4) = (RoCap{i-1}.p(:,2)-RoCap{i-1}.p(:,1))/2;   % mid-point
                % R 
                R=[cos(theta(i-1)) -sin(theta(i-1))  0;
                    sin(theta(i-1)) cos(theta(i-1))  0;
                      0                        0                 1];

                if i == 4
                    Rx=[1     0             0        ;  
                        0  cos(-0.5*pi) -sin(-0.5*pi); 
                        0  sin(-0.5*pi) cos(-0.5*pi) ];

                    R = Rx*R;
                end
                M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1];
                for k=1:size(RoCap{i-1}.p,2)
                pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
                end
                self.check_points = [ pos{i-1}.p(:,4) pos{i-1}.p(:,3) self.check_points];


            end
            
           
           ee = M{i}(1:3,1:3)*[0.141;0;0]+M{i}(1:3,4)+base;
           self.check_points = [ee self.check_points];
           % add base check points
           self.check_points = [self.check_points pos{1}.p(:,5:end)];           
           switch self.SOLVER_info
               case 'base'
                   % base only 
                   self.newNodeXYZ = self.newNode;
               case 'all'
                   % all
                   self.newNodeXYZ = [self.newNode(1:3);ee];
           end
           
%            pp = plot3(self.check_points(1,:),self.check_points(2,:),self.check_points(3,:),'c*');
%            pause
%            delete(pp)
           
       end

           
       
       function self = get_ee(self)
       RoCap = self.sys_info.robot.cap;
        T = self.sys_info.robot.T;
        theta = self.newNode;  

        nlink=size(self.newNode,1);
        M=cell(1,nlink+1); M{1}=eye(4);
        
        for i=2:nlink+1
            RoCap{i-1}.p(:,3) = [0;0;0];
            RoCap{i-1}.p(:,4) = (RoCap{i-1}.p(:,2)-RoCap{i-1}.p(:,1))/2;   % mid-point
            % R 
            R=[cos(theta(i-1)) -sin(theta(i-1))  0;
                sin(theta(i-1)) cos(theta(i-1))  0;
                  0                        0                 1];

            if i == 4
                Rx=[1     0             0        ;  
                    0  cos(-0.5*pi) -sin(-0.5*pi); 
                    0  sin(-0.5*pi) cos(-0.5*pi) ];

                R = Rx*R;
            end
            M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1];
            for k=1:4
            pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+self.sys_info.base;
            end
            self.check_points = [ pos{i-1}.p(:,4) pos{i-1}.p(:,3) self.check_points];
            
            
        end
        self.newNodeXYZ = M{i}(1:3,1:3)*[0.141;0;0]+M{i}(1:3,4)+self.sys_info.base;
        self.all_nodes_XYZ = [self.all_nodes_XYZ self.newNodeXYZ];
        self.check_points = [self.newNodeXYZ self.check_points];

       end
       
     %%%%%%%%%%%%%%%%%%%%%%% Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     function self = plotNode(self)
           figure(self.sys_info.fighandle)
           %plot3(self.newNodeXYZ(1),self.newNodeXYZ(2),self.newNodeXYZ(3),'*r')
           pause(0.01)
     end
           
       
   end
end