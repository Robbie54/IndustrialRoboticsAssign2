clear all
close all
clc
clf

Environment.Run()
Motion.RobotMotion()

%%GUI Panel 
%withing teach there is this 
%RTBPlot.install_teach_panel
%get help on it and have a look to see how teach is being displayed

%potentially can use appdesigner as well however it may be poor
%code/difficult to make it adjust teach to different amount of joints

%potentially use teach as normal then add another panel on other side that
%does cartesian movements