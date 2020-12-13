function [Ainq, binq] = feasible_set(obs,x_,sys_info)
% Ainq*u <= binq

    % get # obstacles
    nobs = sys_info.num_obs;
    H = sys_info.H;
    Aaug = sys_info.Aaug;
    Baug = sys_info.Baug;
    dim = sys_info.dim;
    nstate = sys_info.nstate;
    in_max = sys_info.in_max;
    x0 = x_(1:nstate);
    
    Ainq = [];
    binq = [];
    
    % environmental constraints
    for i = 1:nobs       
        
        for j = 1:H
            x = x_(j*nstate+1:j*nstate+2);
            % g(x*)+dg(x*)'(Ax0+Bu -x*)<=0
            gx = g_f(obs{i}.A,obs{i}.c,obs{i}.m,x);
            dgx = dg_f(obs{i}.A,obs{i}.c,x);
            
            Ainq = [Ainq; dgx'*Baug((j-1)*nstate+1:(j-1)*nstate+2,:)];
            binq = [binq; dgx'*x-gx-dgx'*Aaug((j-1)*nstate+1:(j-1)*nstate+2,:)*x0];
            
        end
        
    end
     
    % input constraints   (uncomment in inter_example) 
    Ainq = [Ainq; kron(eye(H),[eye(dim);-eye(dim)])];
    binq = [binq; kron(ones(H*2,1),in_max)];
    
            
        
end
