function [report skip]= getOrocosDataStruct1(filename, numjoints)
    [data skip] = readOrocosData(filename, 155); %TODO +11
    joints = 1:numjoints;
    cartesian = 1:3;
    
    report = struct;
    report.data = data;
    idx=0;
    report.timestamps                      = data(:,1+idx);%special
    idx=idx+1;
    report.curTrackingpoint                = data(:,cartesian+idx);
    idx=idx+3;
    report.desTrackingpoint                = data(:,cartesian+idx);
    idx=idx+3;
    
    assert(idx+1==size(data,2)); %+1 because of last space as senseless variable
end
