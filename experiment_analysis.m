% function experiment_analysis()

    clc;
    close all;
    clear all;
    
    numjoints = 7;
    [report  skip]  = getOrocosDataStruct(['/home/ndehio/reports.dat'], numjoints);
    
    
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
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('End-Effector translational position per axis')
    pDes=plot(report.timestampsArea, report.desCartPosTaskTranslation(idxArea,1), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTaskTranslation(idxArea,1), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    ylim([-0.7, -0.4])
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTaskTranslation(idxArea,2), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTaskTranslation(idxArea,2), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    ylim([-0.15, 0.15])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartPosTaskTranslation(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curCartPosTaskTranslation(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    
    fig=figure();
    hold all;
    subplot(3,1,1)
    hold all
    title('End-Effector translational velocity per axis')
    pDes=plot(report.timestampsArea, report.desCartVelTaskTranslation(idxArea,1), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTaskTranslation(idxArea,1), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    ylim([-0.15, 0.15])
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desCartVelTaskTranslation(idxArea,2), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTaskTranslation(idxArea,2), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    ylim([-0.15, 0.15])
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desCartVelTaskTranslation(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curCartVelTaskTranslation(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')
    
    fig=figure();
    hold all;
    title('End-Effector position')
    pDes=plot3(report.desCartPosTaskTranslation(idxArea,1),report.desCartPosTaskTranslation(idxArea,2),report.desCartPosTaskTranslation(idxArea,3),'-r');
    pCur=plot3(report.curCartPosTaskTranslation(idxArea,1),report.curCartPosTaskTranslation(idxArea,2),report.curCartPosTaskTranslation(idxArea,3),'-b');
    xlim([-0.70, -0.40])
    ylim([-0.15, +0.15])
    zlim([+0.40, +0.60])
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    axis equal
    legend([pDes,pCur],'desired', 'current')
