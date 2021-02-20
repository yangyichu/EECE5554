imudata1=[xdown(1:1000,:);xup(1:1000,:);ydown(1:1000,:);yup(1:1000,:);zdown(1:1000,:);zup(1:1000,:)];
% imudata1=[xdown;xup;ydown;yup;zdown;zup];
% windowSize = 5; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% imudata=filter(b,a,imudata1);
imudata=imudata1;
% yaw=imudata(:,1);
% pitch=imudata(:,2);
% roll=imudata(:,3);
magx=imudata(:,1);
magy=imudata(:,2);
magz=imudata(:,3);
accx=imudata(:,4);
accy=imudata(:,5);
accz=imudata(:,6);
gyrox=imudata(:,7);
gyroy=imudata(:,8);
gyroz=imudata(:,9);
% plot(accx);
hold on
% plot(accy);
plot(accz);
ylabel('acceleration[m/s2]');
% legend('x-axis','y-axis','z-axis');

%%
% startfrom=550;
% caliend=1200;
% startfrom=1;
% caliend=70171;
% startfrom=6400;
% caliend=9000;
% plot(accz(startfrom:end))
% quiver3(zeros(1192,1),zeros(1192,1),zeros(1192,1),accx(startfrom:caliend),accy(startfrom:caliend),accz(startfrom:caliend));
% gM=[accx,accy,accz,ones(length(imudata),1)];
% g=[ ones(length(xdown),1)*9.8,zeros(length(xdown),1),zeros(length(xdown),1);...
%     ones(length(xup),1)*-9.8,zeros(length(xup),1),zeros(length(xup),1);...
%     zeros(length(ydown),1),ones(length(ydown),1)*9.8,zeros(length(ydown),1);...
%     zeros(length(yup),1),ones(length(yup),1)*-9.8,zeros(length(yup),1);...
%     zeros(length(zdown),1),zeros(length(zdown),1),ones(length(zdown),1)*9.8;...
%     zeros(length(zup),1),zeros(length(zup),1),ones(length(zup),1)*-9.8;...
%     ];
x_axis_offset=mean(accx(401:1600));
y_axis_offset=mean(accy(2401:3600));
z_axis_offset=mean(accz(4401:5600));

% gM=[accx-x_axis_offset,accy-y_axis_offset,accz-z_axis_offset,ones(length(imudata),1)];
% 
% g=[ ones(1000,1)*9.8,zeros(1000,1),zeros(1000,1);...
%     ones(1000,1)*-9.8,zeros(1000,1),zeros(1000,1);...
%     zeros(1000,1),ones(1000,1)*9.8,zeros(1000,1);...
%     zeros(1000,1),ones(1000,1)*-9.8,zeros(1000,1);...
%     zeros(1000,1),zeros(1000,1),ones(1000,1)*9.8;...
%     zeros(1000,1),zeros(1000,1),ones(1000,1)*-9.8;...
%     ];

gM=[accx-x_axis_offset,accy-y_axis_offset,accz-z_axis_offset];

g=[ ones(1000,1)*9.8,zeros(1000,1),zeros(1000,1);...
    ones(1000,1)*-9.8,zeros(1000,1),zeros(1000,1);...
    zeros(1000,1),ones(1000,1)*9.8,zeros(1000,1);...
    zeros(1000,1),ones(1000,1)*-9.8,zeros(1000,1);...
    zeros(1000,1),zeros(1000,1),ones(1000,1)*9.8;...
    zeros(1000,1),zeros(1000,1),ones(1000,1)*-9.8;...
    ];


%%
R=inv(gM'*gM)*gM'*g;
acc=[accx-x_axis_offset,accy-y_axis_offset,accz-z_axis_offset,ones(length(accx),1)];
% plot(acc(:,1));
% plot(acc(:,2));
plot(acc(:,3));
grid on
grid minor

legend('z-axis','z-axis calibrated');
% 
% subplot(311)
% plot(accx(startfrom:caliend)-0.4544);
% subplot(312)
% plot(accy(startfrom:caliend)-0.3842);
% subplot(313)
% plot(accz(startfrom:caliend)-0.2869);