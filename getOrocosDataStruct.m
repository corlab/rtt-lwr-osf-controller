function [report skip]= getOrocosDataStruct1(filename, numjoints, taskSpaceDim)
    joints = 1:numjoints;
    taskSpaceArray = 1:taskSpaceDim;
    [data skip] = readOrocosData(filename, 2+0*numjoints+5*taskSpaceDim);
    
    report = struct;
    report.data = data;
    idx=0;
    report.timestamps           = data(:,1+idx);%special
    idx=idx+1;
    report.curCartPosTask       = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    report.curCartVelTask       = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    report.desCartPosTask       = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    report.desCartVelTask       = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    report.desCartAccTask       = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    
    assert(idx+1==size(data,2)); %+1 because of last space as senseless variable
end
