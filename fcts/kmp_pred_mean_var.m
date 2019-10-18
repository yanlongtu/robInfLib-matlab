function [Mu,Sigma] = kmp_pred_mean_var(t,sampleData,N,kh,Kinv1,Kinv2,lamda2,dim)
% mean: k*inv(K+lamda1*Sigma)*Y and ...
% covariance: num/lamda2*[k(s,s)-k*inv(K+lamda2*Sigma)*k']
k0=kernel_extend(t,t,kh,dim);

D=2*dim;
for i=1:N
    k(1:D,(i-1)*D+1:i*D)=kernel_extend(t,sampleData(i).t,kh,dim);

    for h=1:D
        Y((i-1)*D+h,1)=sampleData(i).mu(h); % [px py ... vx vy ...]'
    end
end
    
Mu=k*Kinv1*Y;% [px py ... vx vy ...]'

Sigma=N/lamda2*(k0-k*Kinv2*k'); 

end

