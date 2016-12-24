function [report skip]= getOrocosDataStructSingleArm(filename, numjoints, taskSpaceDim, cstrSpaceDim)
    joints = 1:numjoints;
    taskSpaceArray = 1:taskSpaceDim;
    cstrSpaceArray = 1:cstrSpaceDim;
    [data skip] = readOrocosData(filename, 2+3*numjoints+5*taskSpaceDim+3*cstrSpaceDim);
    
    report = struct;
    report.data = data;
    idx=0;
    report.timestamps           = data(:,1+idx);%special
    idx=idx+1;
    report.feedback_angles      = data(:,joints+idx);
    idx=idx+numjoints;
    report.feedback_velocities  = data(:,joints+idx);
    idx=idx+numjoints;
    report.feedback_torques     = data(:,joints+idx);
    idx=idx+numjoints;
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
    report.cmdCartForce         = data(:,cstrSpaceArray+idx);
    idx=idx+cstrSpaceDim;
    report.estCartForceA        = data(:,cstrSpaceArray+idx);
    idx=idx+cstrSpaceDim;
    report.estCartForceB        = data(:,cstrSpaceArray+idx);
    idx=idx+cstrSpaceDim;
    
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