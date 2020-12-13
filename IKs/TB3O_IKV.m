function [Ax_next,end_next] = TB3O_IKV(Tx_current,Ax_current,Tx_next,robot,gain,end_hold)

mo2_1 = robot.T(:,3);
% % p_end global
% [end_now,R_]=get_end(Ax_current,Tx_current(1:2),robot);
% % p_end turned wrt 1
% end_now_1 = inv(R_{1})*end_now;
% theta_diff = Tx_next(3)- Tx_current(3);
% Rz(-theta_diff);
% end_next_1 = Rz(-theta_diff)*end_now_1;
%% Inverse Kinematics
% % th2

% Ax_current(2) = atan(m2e_1(2)/m2e_1(1));
% % th 3 4 5
% m2e_1_xz = Rz(-Ax_current(2))*(m2e_1-[0 0 0.04]');
% % flip x2y z2x
% m2e_1_f(2) = m2e_1_xz(1);
% m2e_1_f(1) = m2e_1_xz(3);
% m2e_1_f(3) = 0;
 %% Analytic solution
end_hold_1 = inv(Rz(Tx_next(3)))*(end_hold-[Tx_next(1) Tx_next(2) 0.1]');
m2e_1 = end_hold_1-mo2_1;
[theta_,points] = IK_e2th(m2e_1);
Ax_next = [Tx_next(3); theta_];
[end_next,R_]=get_end(Ax_next,Tx_next(1:2),robot);
end_next;
points(:,end);

if ~(Ax_next == real(Ax_next))
    warning = 1
end
% % velosity method
% v_end_l = (end_next_1-end_now_1)/robot.dt;
% J = J_end_1f(Ax_current(2),Ax_current(3),Ax_current(4),Ax_current(5));
% % inverse weight, big weight small speed
% W = diag([1 1 1 5])/norm([1 1 1 5]);
% Wi = inv(W);
% Jpiw = pinv(J);%inv(J'*J+W)*J';%J'*inv(J*J' + W)% Wi*J'*inv(J*Wi*J');
% VA = Jpiw*v_end_l;
% 
% % next step
% Ax_next = Ax_current(2:5) + robot.dt*VA*(1+gain);
% Ax_next = [Tx_next(3); Ax_next];
% [end_next,R_]=get_end(Ax_next,Tx_next(1:2),robot);
% error = (end_next-end_now);

end