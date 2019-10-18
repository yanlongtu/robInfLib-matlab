function [Kinv] = kmp_estimateMatrix_mean(sampleData,N,kh,lamda,dim)
% calculate: inv(K+lamda*Sigma)
% this function is written for 'dim-' pos and 'dim-' vel

D=2*dim;

for i=1:N
    for j=1:N
        kc((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kernel_extend(sampleData(i).t,sampleData(j).t,kh,dim); 
        
        if i==j
            C_temp=sampleData(i).sigma;           
            kc((i-1)*D+1:i*D,(j-1)*D+1:j*D)=kc((i-1)*D+1:i*D,(j-1)*D+1:j*D)+lamda*C_temp;
        end
    end
end

Kinv=inv(kc); % DN*DN
end

