function [report skip]= getOrocosDataStruct1(filename, numjoints)
    [data skip] = readOrocosData(filename, 2+0*numjoints+5*3);
    joints = 1:numjoints;
    cartesian = 1:3;
    
    report = struct;
    report.data = data;
    idx=0;
    report.timestamps                      = data(:,1+idx);%special
    idx=idx+1;
    report.curCartPosTaskTranslation       = data(:,cartesian+idx);
    idx=idx+3;
    report.curCartVelTaskTranslation       = data(:,cartesian+idx);
    idx=idx+3;
    report.desCartPosTaskTranslation       = data(:,cartesian+idx);
    idx=idx+3;
    report.desCartVelTaskTranslation       = data(:,cartesian+idx);
    idx=idx+3;
    report.desCartAccTaskTranslation       = data(:,cartesian+idx);
    idx=idx+3;
    
    assert(idx+1==size(data,2)); %+1 because of last space as senseless variable
end
