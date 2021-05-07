%-----------demo_KMP_uncertainty----------
% This file provide a simple demo of using kmp, where both the trajectory covariance
% and uncertainty are predicted.
% 
% This code is written by Dr. Yanlong Huang

% @InProceedings{silverio2019uncertainty,
%   Title = {Uncertainty-Aware Imitation Learning using Kernelized Movement Primitives},
%   Author = {Silv\'erio, J. and Huang, Y. and Abu-Dakka, Fares J and Rozo, L. and  Caldwell, D. G.},
%   Booktitle = {Proc. {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
%   Year = {2019, to appear},
% }

%%
clear; close all;
myColors;
addpath('../fcts/');

%% Extract position and velocity from demos
load('../2Dletters/F.mat');
demoNum=5;                    % number of demos
demo_dt=0.01;                 % time interval of data
demoLen=size(demos{1}.pos,2); % size of each demo;
demo_dura=demoLen*demo_dt;    % time length of each demo
dim=2;                        % dimension of demo

totalNum=0;
for i=1:demoNum        
    for j=1:demoLen
        totalNum=totalNum+1;
        Data(1,totalNum)=j*demo_dt;
        Data(2:dim+1,totalNum)=demos{i}.pos(1:dim,j);
    end
    lowIndex=(i-1)*demoLen+1;
    upIndex=i*demoLen;
    for k=1:dim
        Data(dim+1+k,lowIndex:upIndex)=gradient(Data(1+k,lowIndex:upIndex))/demo_dt;
    end    
end

%% Extract the reference trajectory
model.nbStates = 8;   % Number of states in the GMM
model.nbVar =1+2*dim; % Number of variables [t,x1,x2,.. vx1,vx2...]
model.dt = 0.005;     % Time step duration
nbData = demo_dura/model.dt; % Length of each trajectory

model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
[DataOut, SigmaOut] = GMR(model, [1:nbData]*model.dt, 1, 2:model.nbVar); %see Eq. (17)-(19)

for i=1:nbData
    refTraj(i).t=i*model.dt;
    refTraj(i).mu=DataOut(:,i);
    refTraj(i).sigma=SigmaOut(:,:,i);
end

%% Set kmp parameters
dt=0.005;
len=demo_dura/dt;
lamda=1;  % control mean prediction
lamdac=60;% control variance prediction
kh=6;

%% Set desired points
viaFlag=[1 1]; % determine which via-points are used

viaNum=2;
via_time(1)=dt;
via_point(:,1)=[-4 -12 0 0]'; % format:[2D-pos 2D-vel]   
via_time(2)=2;
via_point(:,2)=[8 0 0 0]'; 

via_var=1E-6*eye(4);  % adaptation precision
via_var(3,3)=1000;via_var(4,4)=1000; % low adaptation precision for velocity

%% Update the reference trajectory using desired points
newRef=refTraj;
newLen=len;
for viaIndex=1:viaNum
    if viaFlag(viaIndex)==1       
    [newRef,newLen] = kmp_insertPoint(newRef,newLen,via_time(viaIndex),via_point(:,viaIndex),via_var);
    end
end

%% Prediction mean and variance INSIDE AND OUTSIDE the training region using kmp
[Kinv1, Kinv2] = kmp_estimateMatrix_mean_var(newRef,newLen,kh,lamda,lamdac,dim);

uncertainLen=0.8*len;      % the length of uncertain region
totalLen=len+uncertainLen;

for index=1:totalLen
    t=index*dt;
    [mu sigma]=kmp_pred_mean_var(t,newRef,newLen,kh,Kinv1,Kinv2,lamdac,dim);
    kmpPredTraj(index).t=t;
    kmpPredTraj(index).mu=mu;       
    kmpPredTraj(index).sigma=sigma;    
end

for i=1:totalLen
    kmp(:,i)=kmpPredTraj(i).mu; % format:[2D-pos 2D-vel]
    for h=1:2*dim       
        kmpVar(h,i)=kmpPredTraj(i).sigma(h,h);
        kmpVar(h,i)=sqrt(kmpVar(h,i));
    end 
    SigmaOut_kmp(:,:,i)=kmpPredTraj(i).sigma;    
end


%% Show kmp predictions (mean and covariance/uncertainty)
value=[0.5 0 0.5]; 
curveValue=mycolors.o;

figure('units','normalized','outerposition',[0 0 1 1])
set(gcf, 'Position', [0.1597    0.1311    0.6733    0.2561])

%% plot px-py within the training region
subplot(1,3,1)
plotGMM(kmp(1:2,1:len), SigmaOut_kmp(1:2,1:2,1:len), curveValue, .03);
hold on
plot(kmp(1,1:len),kmp(2,1:len),'color',curveValue,'linewidth',2);
hold on
plot(Data(2,:),Data(3,:),'.','markersize',5,'color','k'); % original data

for viaIndex=1:viaNum
    if viaFlag(viaIndex)==1   
         plot(via_point(1,viaIndex),via_point(2,viaIndex),'o','color',value,'markersize',12,'linewidth',1.5); 
    end
end

box on
xlim([-15 15])
ylim([-15 15])
xlabel('${x}$ [cm]','interpreter','latex');
ylabel('${y}$ [cm]','interpreter','latex');
set(gca,'xtick',[-10 0 10]);
set(gca,'ytick',[-10 0 10]);
set(gca,'FontSize',17)
grid on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;

%% plot t-px and t-py
for plotIndex=1:2
subplot(1,3,1+plotIndex)

% plot original data
hold on
plot(Data(1,1:1:end),Data(1+plotIndex,1:1:end),'.','markersize',5,'color','k'); 

% plot kmp prediction INSIDE the training region
shadowT=dt:dt:dt*len;
shadow_time=[shadowT,fliplr(shadowT)];

shadowUpKmp=kmp(plotIndex,1:len)+kmpVar(plotIndex,1:len);
shadowLowKmp=kmp(plotIndex,1:len)-kmpVar(plotIndex,1:len);
shadow_kmp=[shadowUpKmp,fliplr(shadowLowKmp)];

fill(shadow_time,shadow_kmp',curveValue,'facealpha',.3,'edgecolor','none')
hold on
plot(shadowT,kmp(plotIndex,1:len),'color',curveValue,'linewidth',2.0)

% plot kmp prediction OUTSIDE the training region
shadowT=dt*(len+1):dt:dt*totalLen;
shadow_time=[shadowT,fliplr(shadowT)];

shadowUpKmp=kmp(plotIndex,len+1:totalLen)+kmpVar(plotIndex,len+1:totalLen);
shadowLowKmp=kmp(plotIndex,len+1:totalLen)-kmpVar(plotIndex,len+1:totalLen);
shadow_kmp=[shadowUpKmp,fliplr(shadowLowKmp)];

fill(shadow_time,shadow_kmp',mycolors.y,'facealpha',.3,'edgecolor','none')
hold on
plot(shadowT,kmp(plotIndex,len+1:totalLen),'color',mycolors.y,'linewidth',2.0)

% plot desired points
for viaIndex=1:viaNum
    if viaFlag(viaIndex)==1   
         plot(via_time(viaIndex),via_point(plotIndex,viaIndex),'o','color',value,'markersize',12,'linewidth',1.5); 
    end
end

box on
xlim([0 totalLen*dt])
if plotIndex==1 || plotIndex==2 
    if plotIndex==1
       ylabel('${x}$ [cm]','interpreter','latex');
    end
    if plotIndex==2
       ylabel('${y}$ [cm]','interpreter','latex');
    end  
    ylim([-15 15])
    set(gca,'ytick',[-10 0 10]);
end

xlabel('$t$ [s]','interpreter','latex');
set(gca,'xtick',[0 1 2]);
set(gca,'FontSize',17)
grid on
set(gca,'gridlinestyle','--')
ax=gca;
ax.GridAlpha=0.3;
end

