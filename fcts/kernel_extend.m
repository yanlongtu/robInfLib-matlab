function [kernelMatrix] = kernel_extend(ta,tb,h,dim)
% this file is used to generate a kernel value
% this kernel considers the 'dim-' pos and 'dim-' vel

%% callculate different kinds of kernel
dt=0.001;
tadt=ta+dt;
tbdt=tb+dt;

kt_t=exp(-h*(ta-tb)*(ta-tb)); % k(ta,tb)

kt_dt_temp=exp(-h*(ta-tbdt)*(ta-tbdt)); % k(ta,tb+dt)
kt_dt=(kt_dt_temp-kt_t)/dt; % (k(ta,tb+dt)-k(ta,tb))/dt

kdt_t_temp=exp(-h*(tadt-tb)*(tadt-tb)); % k(ta+dt,tb)
kdt_t=(kdt_t_temp-kt_t)/dt; % (k(ta+dt,tb)-k(ta,tb))/dt

kdt_dt_temp=exp(-h*(tadt-tbdt)*(tadt-tbdt)); % k(ta+dt,tb+dt)
kdt_dt=(kdt_dt_temp-kt_dt_temp-kdt_t_temp+kt_t)/dt/dt; % (k(ta+dt,tb+dt)-k(ta,tb+dt)-k(ta+dt,tb)+k(ta,tb))/dt/dt

kernelMatrix=zeros(2*dim,2*dim);
for i=1:dim
    kernelMatrix(i,i)=kt_t;
    kernelMatrix(i,i+dim)=kt_dt;
    kernelMatrix(i+dim,i)=kdt_t;
    kernelMatrix(i+dim,i+dim)=kdt_dt;
end

end

