function [X_out, u] = base_ILQR(states_out,var)

%% ILQR
    NILQR = var.N;
    traj=[];
    if isempty(states_out)==false
        nstate = var.nstate;
        traj(1,:) = states_out(1:nstate:end)';
        traj(2,:) = states_out(2:nstate:end)';
        traj(3,:) = states_out(3:nstate:end)';
        traj(4,:) = states_out(4:nstate:end)';
        traj(5,:) = states_out(5:nstate:end)';
        Tx_current = states_out(:,1);
    end
    

    
    xref_ = traj(:,1:NILQR);
     
    
    
    L1 = zeros(7,NILQR);
    
    %L2 = diag([1000 1000 0 0 0 1 1 ]);
    %L2 = diag([10 100 0 0 0 1 0.1 ]);   % normal
    L2 = diag([200 200 100 0 0 100 10 ]);
    %L2 = diag([1000 1000 0 0 0 100 10 ]); % for tpa_cfs_MMD_0902
    %L2 = diag([1000 1000 100 0 0 10 10 ]); % for center (fix ee)
    L2=reshape(repmat(L2,1,NILQR),7,7,NILQR);
    l=zeros(1,NILQR);
    
    u0 = zeros(2,NILQR-1);
    

    %xref_
    [ X_out, u, xbar, ubar, kk, K, sigsu, A, B ] = MaxEntILQR( L2, L1, l, Tx_current, u0, var, xref_ );

    
end