function [end_all] = plot_MM5(ss,theta_implement,traj_implement,robot,tb3,gap,plot_mode,v,fighandle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ploting function for TB3O
% theta_implement 5 x ss
% traj_implement: 2 x ss
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pos={};
plotbase2=[];
plotbase1=[];
plotwheels=[];
s1= [];
links_=[];
etrj=[];
plotdoor=[];
end_all = [];



%% Plot
cons_c = 1;
for i=1:gap:ss
    
    delete(plotbase2)
    delete(plotbase1)
    delete(plotwheels)
    delete(links_)
    delete(etrj)
    delete(s1)
    
    
    % get bace position    
    base = [traj_implement(:,i); 0.1];
    % get reference theta
    theta=theta_implement(:,i);
    [pos,M]=plot_MMlink(theta,base',robot);
    [pend,R_]=get_end(theta,base(1:2),robot);
    
    % plot base
    tb3_base_w = M{2}(1:2,1:2)*tb3.base+base(1:2);
    plotbase2 =fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.1*ones(1,4), ...
        [0.9-cons_c/1.2,0.9-cons_c/1.2,0.9-cons_c/1.2]);
    hold on
    plotbase1 = fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.05*ones(1,4),...
        [0.9-cons_c/1.2,0.9-cons_c/1.2,0.9-cons_c/1.2]);
    
    
    % plot wheels
    for w = 1:2
    tb3_w_w = M{2}(1:2,1:2)*[tb3.wx(w,:);tb3.wy(w,:)]+base(1:2);
    plotwheels(w) = fill3(tb3_w_w(1,:), tb3_w_w(2,:), tb3.wz(w,:),...
        [0.9-cons_c/1.2,0.9-cons_c/1.2,0.9-cons_c/1.2]);
    end
    
    % plot arm
    for j = 1:6
        if j<6
       links_(j) = plot3([ pos{j}.p(1,3), pos{j+1}.p(1,3)],[ pos{j}.p(2,3), pos{j+1}.p(2,3)],[ pos{j}.p(3,3), pos{j+1}.p(3,3)],'k-','color',[1-cons_c,1-cons_c,1-cons_c/1.5],'LineWidth',3);
        end  
       
       etrj(j) = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),'o-','color',[1-cons_c/3.5,1-cons_c/2.5,1-cons_c],'LineWidth',3);
       hold on
    end
    
    end_all = [end_all pos{j}.p(1:3,3)];    
    xlabel(['x[m] Time step:' num2str(i)])
    ylabel('y[m]')
    zlabel('z[m]')
    axis equal
    axis([robot.view_area])
    view([2 1 5.5])  %%
    if ~isempty(v)
        frame = getframe(fighandle(1));
        writeVideo(v,frame);
    end
    % plot door
    switch(plot_mode) 
        
        case 0       %% just MM for animation of motion planning
            pause(0.1*gap)
            continue   
        case 1      %% MM and Obj for animation of pose adapting
            [x y z] = sphere;
            r_obj = 0.03;
            objc = pos{6}.p(:,3) + r_obj*(pos{6}.p(:,3)-pos{5}.p(:,3))/norm(pos{6}.p(:,3)-pos{5}.p(:,3));
            a=[objc' r_obj];
            s1=surf(x*a(1,4)+a(1,1),y*a(1,4)+a(1,2),z*a(1,4)+a(1,3));
            s1.EdgeColor = 'cyan';
            s1.FaceColor = 'cyan';
            pause(0.05)        
        
    end   
    
end

   

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')

axis equal
axis(robot.view_area)

end