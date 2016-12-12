function [report skip]= getOrocosDataStruct1(filename, numjoints, taskSpaceDim)
    joints = 1:numjoints;
    taskSpaceArray = 1:taskSpaceDim;
    [data skip] = readOrocosData(filename, 2+0*numjoints+6*taskSpaceDim);
    
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
    report.cmdCartForce         = data(:,taskSpaceArray+idx);
    idx=idx+taskSpaceDim;
    
    assert(idx+1==size(data,2)); %+1 because of last space as senseless variable
    
    if taskSpaceDim==6
        report.curCartPosTaskQuat = AxisAngle2Quaternion(report.curCartPosTask(:,4:6));
        report.curCartVelTaskQuat = AxisAngle2Quaternion(report.curCartVelTask(:,4:6));
        report.desCartPosTaskQuat = AxisAngle2Quaternion(report.desCartPosTask(:,4:6));
        report.desCartVelTaskQuat = AxisAngle2Quaternion(report.desCartVelTask(:,4:6));
        report.desCartAccTaskQuat = AxisAngle2Quaternion(report.desCartAccTask(:,4:6));
    end
end

function quat = AxisAngle2Quaternion(axisangle)
    quat(:,1) = 0;
    quat(:,2) = 0;
    quat(:,3) = 0;
    quat(:,4) = 0;
    %TODO...
end