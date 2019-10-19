function demo_KMP_orientation
% This file provide a simple demo of using orientation kmp, where orientation adaptations towards
% various desired points in terms of quaternions and angular velocities are studied.
% 
% This code is written by Dr. Yanlong Huang

% @InProceedings{Huang19ICRA_2,
%   Title = {Generalized Orientation Learning in Robot Task Space},
%   Author = {Huang, Y. and Abu-Dakka, F. and Silv\'erio, J. and Caldwell, D. G.},
%   Booktitle = {Proc. {International Conference on Robotics and Automation ({ICRA})},
%   Year = {2019},
%   Address = {Montreal, Canada},
%   Month = {May},
%   pages = {2531--2537}
% }
 
% @InProceedings{Huang19IJRR,
%   Title = {Kernelized Movement Primitives},
%   Author = {Huang, Y. and Rozo, L. and Silv\'erio, J. and Caldwell, D. G.},
%   Booktitle = {International Journal of Robotics Research},
%   Year= {2019},
%   pages= {833--852}
% }

% @InProceedings{Huang19ICRA_1,
%   Title = {Non-parametric Imitation Learning of Robot Motor Skills},
%   Author = {Huang, Y. and Rozo, L. and Silv\'erio, J. and Caldwell, D. G.},
%   Booktitle = {Proc. {International Conference on Robotics and Automation ({ICRA})},
%   Year = {2019},
%   Address = {Montreal, Canada},
%   Month = {May},
%   pages = {5266--5272}
% }

%%
myColors;
addpath('../fcts/');

%% Generate synthetic demos(quaternions)
tau=10;
dt=0.01;
len=round(tau / dt);

tau1=0.4*tau;
tau2=tau-tau1;
N1=tau1/dt;
N2=len-N1;
demosAll=[];   % save demos
demoNum=5;
for num=1:demoNum   
    a=[1 1.2 1 1.5];
    b=[2 3.4 1.5 4]+[0.5 -0.1 1 -0.2]*num/5*0.5;
    c=[4 3 2.5 3]+[0.5 -0.0 -0.1 -1.5]*num/5*0.2;
    q1_ini=quatnormalize(a);
    q2_mid=quatnormalize(b);
    q1.s=q1_ini(1); q1.v=q1_ini(2:4)';
    q2.s=q2_mid(1); q2.v=q2_mid(2:4)'; 
    q3_end=quatnormalize(c);
    q3.s=q3_end(1); q3.v=q3_end(2:4)'; 
    
    [q, omega, domega, t] = generate_orientation_data(q1, q2, tau1, dt); % generate quaternion data (first part)
    for i=1:N1
        demo(1,i)  = i*dt;
        demo(2,i)  = q(i).s;
        demo(3:5,i)= q(i).v;
        demo(6:8,i)= omega(:,i);
    end
    [q, omega, domega, t] = generate_orientation_data(q2, q3, tau2, dt); % generate quaternion data (second part)
    for i=1:N2
        demo(1,i+N1)  = i*dt+N1*dt;
        demo(2,i+N1)  = q(i).s;
        demo(3:5,i+N1)= q(i).v;
        demo(6:8,i+N1)= omega(:,i);
    end 
    demosAll=[demosAll demo];   
end

%% Project demos into Euclidean space and model new trajectories using GMM/GMR
Data=[]; % save transformed data
for j=1:demoNum*len
    time=demosAll(1,j);
    qtemp.s=demosAll(2,j);
    qtemp.v=demosAll(3:5,j);

    zeta(1,j)=time;
    zeta(2:4,j)=quat_log(qtemp, q1);  %q1 serves as the auxiliary quaternion
end
for h=1:3        
zeta(4+h,:)=gradient(zeta(h+1,:))/dt;
end
Data=[Data zeta];

model.nbStates = 5; %Number of states in the GMM
model.nbVar =7;     %Number of variables [t,wx,wy,wz,ax,ay,az]
model.dt = 0.01;    %Time step duration
nbData = 1000;      %Length of each trajectory

model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
[DataOut, SigmaOut] = GMR(model, [1:nbData]*model.dt, 1, 2:model.nbVar); 

for i=1:nbData
    quatRef(i).t=i*model.dt;
    quatRef(i).mu=DataOut(:,i);
    quatRef(i).sigma=SigmaOut(:,:,i);
end

%% Set kmp parameters
lamda=1;
kh=0.01;
dim=3;

%% Set desired quaternion and angular velocity
viaFlag=[1 1 1]; % specify desired point
% viaFlag=[0 0 0]; % reproduction

via_time(1)=0;   % desired time
qDes_temp(:,1)=q1_ini;
qDes1.s=qDes_temp(1,1); qDes1.v=qDes_temp(2:4,1); % desired quaternion
dqDes(:,1)=[0.0 0.0 0.0]'; % desire angular velocity

via_time(2)=4.0; 
des_temp2=[0.2 0.6 0.4 0.8];
qDes_temp(:,2)=quatnormalize(des_temp2);
qDes2.s=qDes_temp(1,2); qDes2.v=qDes_temp(2:4,2); 
dqDes(:,2)=[0.1 0.1 0.0]';

via_time(3)=10.0;  
des_temp3=[0.6 0.4 0.6 0.4];
qDes_temp(:,3)=quatnormalize(des_temp3); 
qDes3.s=qDes_temp(1,3); qDes3.v=qDes_temp(2:4,3); 
dqDes(:,3)=[0 0.3 0.3]'; 

%% Transform desired points into Euclidean space
via_point(1:3,1)=quat_log(qDes1, q1);
via_point(4:6,1)=trans_angVel(qDes1, dqDes(:,1), dt, q1); 
via_point(1:3,2)=quat_log(qDes2, q1);
via_point(4:6,2)=trans_angVel(qDes2, dqDes(:,2), dt, q1); 
via_point(1:3,3)=quat_log(qDes3, q1);
via_point(4:6,3)=trans_angVel(qDes3, dqDes(:,3), dt, q1); 

via_var=1E-10*eye(6);
% via_var(4,4)=1000;via_var(5,5)=1000;via_var(6,6)=1000;

%% Update the reference trajectory using transformed desired points
interval=5; % speed up the computation
num=round(len/interval)+1;
for i=1:num
    if i==1 index=1;
    else index=(i-1)*interval;
    end
    sampleData(i).t=quatRef(index).t;
    sampleData(i).mu=quatRef(index).mu;
    sampleData(i).sigma=quatRef(index).sigma; 
end

for i=1:3
if viaFlag(i) [sampleData,num] = kmp_insertPoint(sampleData,num,via_time(i),via_point(:,i),via_var);end
end

%% KMP prediction
Kinv = kmp_estimateMatrix_mean(sampleData,num,kh,lamda,dim);

for index=1:len
    t=index*dt;
    mu=kmp_pred_mean(t,sampleData,num,kh,Kinv,dim);    
    kmpTraj(1,index)=t;
    kmpTraj(2:7,index)=mu;
end

%% Project predicted trajectory from Euclidean space into quaternion space
for i=1:len
    qnew=quat_exp(kmpTraj(2:4,i));
    trajAdaQuat(i)=quat_mult(qnew,q1);   % save final predicted quaternion
    
    trajAda(1,i)=kmpTraj(1,i);
    trajAda(2,i)=trajAdaQuat(i).s;
    trajAda(3:5,i)=trajAdaQuat(i).v;
end
[ adaOmega, adaDomega ] = quat_to_vel(trajAdaQuat, dt, tau); % estimate angular vel/acc

%% Show demonstrations
figure
set(gcf, 'Position', [695 943 1350 425])
subplot(1,2,1)
hold on
plot(demosAll(1,1:1:end),demosAll(2:5,1:1:end),'.') 
xlabel('t [s]','interpreter','tex')
ylabel('  ','interpreter','tex')
ylim([0 1])
set(gca,'xtick',[0 5 10])
set(gca,'ytick',[0 0.5 1])
set(gca,'FontSize',18)
grid on
box on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;
legend({'$q_s$','$q_x$','$q_y$','$q_z$'},'interpreter','latex','Orientation','horizontal','FontSize',20)

subplot(1,2,2)
hold on
plot(demosAll(1,1:1:end),demosAll(6:8,1:1:end),'.') 
xlabel('t [s]','interpreter','tex')
ylabel('  [rad/s]','interpreter','tex')
ylim([-0.5 0.5])
set(gca,'xtick',[0 5 10])
set(gca,'ytick',[-0.5 0 0.5])
set(gca,'FontSize',18)
grid on
box on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;
legend({'$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter','latex','Orientation','horizontal','FontSize',20)
        
%% Show kmp predictions
figure
set(gcf, 'Position', [690 384 1357 425])
subplot(1,2,1)
plot(trajAda(1,:),trajAda(2:5,:),'linewidth',3.0) % plot quaternion trajectory
for plotIndex=1:4
    for viaIndex=1:3
        if viaFlag(viaIndex)==1
         hold on
         plot(via_time(viaIndex),qDes_temp(plotIndex,viaIndex),'o','color',mycolors.nr,'markersize',8,'linewidth',1.5); 
        end
    end
end

xlabel('t [s]','interpreter','tex')
ylabel('  ','interpreter','tex')
ylim([0 1])
set(gca,'xtick',[0 5 10])
set(gca,'ytick',[0 0.5 1])
set(gca,'FontSize',18)
grid on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;
legend({'$q_s$','$q_x$','$q_y$','$q_z$'},'interpreter','latex','Orientation','horizontal','FontSize',20)

subplot(1,2,2)
plot(trajAda(1,:),adaOmega(1:3,:),'linewidth',3.0) % plot angular velocity
for plotIndex=1:3
    for viaIndex=1:3
        if viaFlag(viaIndex)==1 
         hold on
         plot(via_time(viaIndex),dqDes(plotIndex,viaIndex),'o','color',mycolors.nr,'markersize',8,'linewidth',1.5); 
        end
    end
end

xlabel('t [s]','interpreter','tex')
ylabel('  [rad/s]','interpreter','tex')
ylim([-0.5 0.5])
set(gca,'xtick',[0 5 10])
set(gca,'ytick',[-0.5 0 0.5])
set(gca,'FontSize',18)
grid on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;
legend({'$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter','latex','Orientation','horizontal','FontSize',20)

