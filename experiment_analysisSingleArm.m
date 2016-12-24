% function experiment_analysisSingleArm()
    %this matlab file plots data logged by one of the following ops files:
    % - full_controller_boris_base.ops
    % - full_controller_boris_base2.ops
    % - full_controller_boris_base_normal.ops
    
    clc;
    close all;
    clear all;
    
    taskSpaceDim = 3; %for Translation only
    taskSpaceDim = 6; %for Translation and Orientation
    cstrSpaceDim = taskSpaceDim; %for Translation and Orientation
    numjoints = 7;
    [report  skip]  = getOrocosDataStructSingleArm(['/home/nde/reports.dat'], numjoints, taskSpaceDim, cstrSpaceDim);
    
    
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
    desJointAngles(1) = 0.72;
    desJointAngles(2) = 1.12;
    desJointAngles(3) = 0.59;
    desJointAngles(4) =-1.31;
    desJointAngles(5) =-2.11;
    desJointAngles(6) = 1.32;
    desJointAngles(7) = 0.00;
    
    fig=figure();
    hold all;
    for jointID=1:1:numjoints
        subplot(numjoints,1,jointID)
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
    title('End-Effector translational position per axis')
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
    ylim([0.0, 0.7])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    ylim([0.7, 0.9])
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    
    if taskSpaceDim == 6
        fig=figure();
        hold all;
        subplot(3,1,1)
        hold all
        title('End-Effector orientational position (Euler)')
        pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,4), '-r');
        pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,4), '-b');
        ylabel('x axis')
        xlim(xLimit)
        subplot(3,1,2)
        hold all
        pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,5), '-r');
        pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,5), '-b');
        ylabel('y axis')
        xlim(xLimit)
        subplot(3,1,3)
        hold all
        pDes=plot(report.timestampsArea, report.desCartPosTask(idxArea,6), '-r');
        pCur=plot(report.timestampsArea, report.curCartPosTask(idxArea,6), '-b');
        ylabel('z axis')
        xlim(xLimit)
        xlabel('Time [sec]')
        legend([pDes,pCur],'desired', 'current')
        
%         fig=figure();
%         hold all;
%         subplot(4,1,1)
%         hold all
%         title('End-Effector orientational position (Quaternion)')
%         pDes=plot(report.timestampsArea, report.desCartPosTaskQuat(idxArea,1), '-r');
%         pCur=plot(report.timestampsArea, report.curCartPosTaskQuat(idxArea,1), '-b');
%         ylabel('x')
%         xlim(xLimit)
%         subplot(4,1,2)
%         hold all
%         pDes=plot(report.timestampsArea, report.desCartPosTaskQuat(idxArea,2), '-r');
%         pCur=plot(report.timestampsArea, report.curCartPosTaskQuat(idxArea,2), '-b');
%         ylabel('y')
%         xlim(xLimit)
%         subplot(4,1,3)
%         hold all
%         pDes=plot(report.timestampsArea, report.desCartPosTaskQuat(idxArea,3), '-r');
%         pCur=plot(report.timestampsArea, report.curCartPosTaskQuat(idxArea,3), '-b');
%         ylabel('z')
%         xlim(xLimit)
%         subplot(4,1,4)
%         hold all
%         pDes=plot(report.timestampsArea, report.desCartPosTaskQuat(idxArea,4), '-r');
%         pCur=plot(report.timestampsArea, report.curCartPosTaskQuat(idxArea,4), '-b');
%         ylabel('w')
%         xlim(xLimit)
%         xlabel('Time [sec]')
%         legend([pDes,pCur],'desired', 'current')
    end
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('End-Effector translational velocity per axis')
    pDes=plot(report.timestampsArea, report.desCartVelTask(idxArea,1), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTask(idxArea,1), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    ylim([-0.15, 0.15])
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desCartVelTask(idxArea,2), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTask(idxArea,2), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    ylim([-0.15, 0.15])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartVelTask(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTask(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('End-Effector translational force per axis')
    pCMD=plot(report.timestampsArea, report.cmdCartForce(idxArea,1), '-r');
    pESTA=plot(report.timestampsArea, report.estCartForceA(idxArea,1), '-b');
    pESTB=plot(report.timestampsArea, report.estCartForceB(idxArea,1), '-g');
    ylabel('x axis [Nm]')
    xlim(xLimit)
    ylim([-20, +20])
    subplot(3,1,2)
    hold all
    pCMD=plot(report.timestampsArea, report.cmdCartForce(idxArea,2), '-r');
    pESTA=plot(report.timestampsArea, report.estCartForceA(idxArea,2), '-b');
    pESTB=plot(report.timestampsArea, report.estCartForceB(idxArea,2), '-g');
    ylabel('y axis [Nm]')
    xlim(xLimit)
    ylim([-20, +20])
    subplot(3,1,3)
    hold all
    pCMD=plot(report.timestampsArea, report.cmdCartForce(idxArea,3), '-r');
    pESTA=plot(report.timestampsArea, report.estCartForceA(idxArea,3), '-b');
    pESTB=plot(report.timestampsArea, report.estCartForceB(idxArea,3), '-g');
    ylabel('z axis [Nm]')
    xlim(xLimit)
    ylim([30, 50])
    xlabel('Time [sec]')
    legend([pCMD,pESTA,pESTB],'commanded via positionctrl','estim. A via constraint', 'estim. B via constraint')
    
    
    if taskSpaceDim == 6
        fig=figure();
        hold all;
        subplot(3,1,1)
        hold all
        title('End-Effector orientational force')
        p=plot(report.timestampsArea, report.cmdCartForce(idxArea,4), '-r');
        ylabel('x axis')
        xlim(xLimit)
        ylim([-20, +20])
        subplot(3,1,2)
        hold all
        p=plot(report.timestampsArea, report.cmdCartForce(idxArea,5), '-r');
        ylabel('y axis')
        xlim(xLimit)
        ylim([-20, +20])
        subplot(3,1,3)
        hold all
        p=plot(report.timestampsArea, report.cmdCartForce(idxArea,6), '-r');
        ylabel('z axis')
        xlim(xLimit)
        ylim([-20, +20])
        xlabel('Time [sec]')
        legend([p],'command')
    end
    
    fig=figure();
    hold all;
    title('End-Effector position in 3D')
    pDes=plot3(report.desCartPosTask(idxArea,1),report.desCartPosTask(idxArea,2),report.desCartPosTask(idxArea,3),'-r');
    pCur=plot3(report.curCartPosTask(idxArea,1),report.curCartPosTask(idxArea,2),report.curCartPosTask(idxArea,3),'-b');
%     xlim([-0.70, -0.40])
%     ylim([-0.15, +0.15])
%     zlim([+0.40, +0.60])
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    axis equal
    legend([pDes,pCur],'desired', 'current')
