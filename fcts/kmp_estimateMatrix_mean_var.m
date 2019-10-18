function [Kinv1,Kinv2] = kmp_estimateMatrix_mean_var(sampleData,N,kh,lamda1,lamda2,dim)
% calculate: inv(K+lamda1*Sigma) and inv(K+lamda2*Sigma)and
% this function is written for 'dim-' pos and 'dim-' vel

D=2*dim;

for i=1:N
    for j=1:N
        kc1((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kernel_extend(sampleData(i).t,sampleData(j).t,kh,dim); 
        kc2((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kc1((i-1)*D+1:i*D,(j-1)*D+1:j*D);
        
        if i==j
            C_temp=sampleData(i).sigma;            
            kc1((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kc1((i-1)*D+1:i*D,(j-1)*D+1:j*D)+lamda1*C_temp;
            kc2((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kc2((i-1)*D+1:i*D,(j-1)*D+1:j*D)+lamda2*C_temp;
        end
    end
end

Kinv1=inv(kc1); 
Kinv2=inv(kc2); 
end

