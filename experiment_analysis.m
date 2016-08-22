% function experiment_analysis()

    clc;
    close all;
    clear all;
    
    numjoints = 7;
    [report  skip]  = getOrocosDataStruct(['/home/ndehio/reports.dat'], numjoints);
    
    
    mystarttime = 1.0;
    mystoptime  = 61.0;
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
    title('End-Effector position per axis')
    pDes=plot(report.timestampsArea, report.desTrackingpoint(idxArea,1), '-r');
    pCur=plot(report.timestampsArea, report.curTrackingpoint(idxArea,1), '-b');
    ylabel('x axis [m]')
    xlim(xLimit)
    subplot(3,1,2)
    hold all
    pDes=plot(report.timestampsArea, report.desTrackingpoint(idxArea,2), '-r');
    pCur=plot(report.timestampsArea, report.curTrackingpoint(idxArea,2), '-b');
    ylabel('y axis [m]')
    xlim(xLimit)
    subplot(3,1,3)
    hold all
    pDes=plot(report.timestampsArea, report.desTrackingpoint(idxArea,3), '-r');
    pCur=plot(report.timestampsArea, report.curTrackingpoint(idxArea,3), '-b');
    ylabel('z axis [m]')
    xlim(xLimit)
    xlabel('Time [sec]')
    legend([pDes,pCur],'desired', 'current')


