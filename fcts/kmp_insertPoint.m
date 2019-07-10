function [newData,newNum] = kmp_insertPoint(data,num,via_time,via_point,via_var)
% insert data format:[time px py ... vx vy ...]';

newData=data;
newNum=num;

dataExit=0;
for i=1:newNum
    if abs(newData(i).t-via_time)<0.0005
        dataExit=1;
        replaceNum=i;
        break
    end
end
if dataExit
    newData(replaceNum).t=via_time;
    newData(replaceNum).mu=via_point;
    newData(replaceNum).sigma=via_var;  
else
    newNum=newNum+1;
    newData(newNum).t=via_time;
    newData(newNum).mu=via_point;
    newData(newNum).sigma=via_var;  
end


end

