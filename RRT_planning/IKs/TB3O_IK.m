function [Ax_next,end_next] = TB3O_IK(Tx_next,robot,end_hold)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end_hold 3x1
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mo2_1 = robot.T(:,3);

%% Inverse Kinematics
% Analytic solution
end_hold_1 = inv(Rz(Tx_next(3)))*(end_hold-[Tx_next(1) Tx_next(2) 0.1]');
m2e_1 = end_hold_1-mo2_1;
[theta_,points] = IK_e2th(m2e_1);
Ax_next = [Tx_next(3); theta_];
[end_next,R_]=get_end(Ax_next,Tx_next(1:2),robot);
end_next;
%%% angle in R+ domain
if Ax_next(1)<0
    Ax_next(1) = Ax_next(1)+pi*2;
end
if Ax_next(2)<0
    Ax_next(2) = Ax_next(2)+pi*2;
end
%%%


points(:,end);

end