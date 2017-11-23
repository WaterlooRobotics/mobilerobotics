function [Sxy_u] = CrossCovariance(Sxy_u,n,w_c,x_sp,mup_u,y_sm,y_u )
%Finds the covariance from measurement and motion model
%Sxy_u: cross covariance, n: number of states, w_c: weighted covariance,
%x_sp: 
    for i=1:2*n+1
        Sxy_u = Sxy_u + w_c(i)*((x_sp(:,i)-mup_u)*(y_sm(:,i)-y_u)');
    end

end

