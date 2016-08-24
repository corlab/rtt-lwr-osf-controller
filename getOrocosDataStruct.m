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
    
    if taskSpaceDim==6
        report.curCartPosTaskQuat = EulerToQuaternion(report.curCartPosTask(:,4:6));
        report.curCartVelTaskQuat = EulerToQuaternion(report.curCartVelTask(:,4:6));
        report.desCartPosTaskQuat = EulerToQuaternion(report.desCartPosTask(:,4:6));
        report.desCartVelTaskQuat = EulerToQuaternion(report.desCartVelTask(:,4:6));
        report.desCartAccTaskQuat = EulerToQuaternion(report.desCartAccTask(:,4:6));
    end
end

function quat = EulerToQuaternion(euler)
	%compute x
    quat(:,1) = (sin(euler(:,1) / 2) .* cos(euler(:,2) / 2) .* cos(euler(:,3) / 2)) ...
			- (cos(euler(:,1) / 2) .* sin(euler(:,2) / 2) .* sin(euler(:,3) / 2));
	%compute y
    quat(:,2) = (cos(euler(:,1) / 2) .* sin(euler(:,2) / 2) .* cos(euler(:,3) / 2)) ...
			+ (sin(euler(:,1) / 2) .* cos(euler(:,2) / 2) .* sin(euler(:,3) / 2));
	%compute z
    quat(:,3) = (cos(euler(:,1) / 2) .* cos(euler(:,2) / 2) .* sin(euler(:,3) / 2)) ...
			- (sin(euler(:,1) / 2) .* sin(euler(:,2) / 2) .* cos(euler(:,3) / 2));
	%compute w
    quat(:,4) = (cos(euler(:,1) / 2) .* cos(euler(:,2) / 2) .* cos(euler(:,3) / 2)) ...
			+ (sin(euler(:,1) / 2) .* sin(euler(:,2) / 2) .* sin(euler(:,3) / 2));
end