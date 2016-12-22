% function experiment_analysisDualArm()
    %this matlab file plots data logged by one of the following ops files:
    % - full_controller_boris_base2DualArm.ops
    
    clc;
    close all;
    clear all;
    
    taskSpaceDim = 12; %for Translation and Orientation
    numjoints = 14;
    [report  skip]  = getOrocosDataStructDualArm(['/home/nde/reports.dat'], numjoints, taskSpaceDim);
    
    
    mystarttime = 0.0;
    mystoptime  = 9.0;
    mystoptime  = floor(report.timestamps(end));
    idxStart =find(report.timestamps>mystarttime,1);
    idxStop =find(report.timestamps>mystoptime,1);
    idxArea =idxStart:idxStop;
    report.timestampsArea =  report.timestamps(idxArea);
    report.timestampsArea =  report.timestampsArea  - mystarttime;

    
    %%
    xLimit = [0.0, mystoptime - mystarttime];
    
    %left arm
    desJointAngles(1) = 0.461369;
    desJointAngles(2) = 1.84417;
    desJointAngles(3) =-1.54299;
    desJointAngles(4) = 1.34663;
    desJointAngles(5) = 1.55174;
    desJointAngles(6) = 1.47731;
    desJointAngles(7) = 0.0;

    %right arm
    desJointAngles(8) =-0.46589;
    desJointAngles(9) = 1.84609;
    desJointAngles(10)=-1.60562;
    desJointAngles(11)=-1.35433;
    desJointAngles(12)= 1.58999;
    desJointAngles(13)= 1.4589;
    desJointAngles(14)= 0.0;
    
    fig=figure();
    hold all;
    for jointID=1:1:numjoints/2
        subplot(numjoints/2,1,jointID)
        hold all
        title(['joint ' num2str(jointID)])
        plot(report.timestampsArea, report.feedback_angles(idxArea,jointID), '-r');
        plot([report.timestampsArea(1) report.timestampsArea(end)], [desJointAngles(jointID) desJointAngles(jointID)], '-k');
        ylabel('angle [rad]')
        xlim(xLimit)
        ylim([-pi, pi])
    end
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('Left-End-Effector translational position per axis')
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,1), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,1), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    ylim([-0.7, -0.4])
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,2), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,2), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    ylim([0.0, 0.6])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    ylim([0.7, 1.5])
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('Right-End-Effector translational position per axis')
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,7), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,7), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    ylim([-0.7, -0.4])
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,8), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,8), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    ylim([-0.6, 0.0])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,9), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,9), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    ylim([0.7, 1.5])
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    

    

    yLimits=[-40, +40];
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('Left-End-Effector translational force per axis')
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,1), '-r');
    ylabel('x axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    subplot(3,1,2)
    hold all
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,2), '-r');
    ylabel('y axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    subplot(3,1,3)
    hold all
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,3), '-r');
    ylabel('z axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    xlabel('Time [sec]')
    legend([p],'command')
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('Right-End-Effector translational force per axis')
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,7), '-r');
    ylabel('x axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    subplot(3,1,2)
    hold all
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,8), '-r');
    ylabel('y axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    subplot(3,1,3)
    hold all
    p=plot(report.timestampsArea, report.cmdCartForce(idxArea,9), '-r');
    ylabel('z axis [Nm]')
    xlim(xLimit)
    ylim(yLimits)
    xlabel('Time [sec]')
    legend([p],'command')
    
    
    fig=figure();
    hold all;
    title('End-Effector position in 3D')
    pDesL=plot3(report.desCartPosTask(idxArea,1),report.desCartPosTask(idxArea,2),report.desCartPosTask(idxArea,3),'-r');
    pCurL=plot3(report.curCartPosTask(idxArea,1),report.curCartPosTask(idxArea,2),report.curCartPosTask(idxArea,3),'-b');
    pDesR=plot3(report.desCartPosTask(idxArea,7),report.desCartPosTask(idxArea,8),report.desCartPosTask(idxArea,9),'-g');
    pCurR=plot3(report.curCartPosTask(idxArea,7),report.curCartPosTask(idxArea,8),report.curCartPosTask(idxArea,9),'-m');
%     xlim([-0.70, -0.40])
%     ylim([-0.15, +0.15])
%     zlim([+0.40, +0.60])
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    axis equal
    legend([pDesL,pCurL,pDesR,pCurR],'desired left', 'current left', 'desired right', 'current right')
