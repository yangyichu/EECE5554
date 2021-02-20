% clear all
% load('sensordata')
load('imugps22');
% x=gpsdata(:,3)-gpsdata(1,3);
% y=gpsdata(:,4)-gpsdata(1,4);
% yaw=imudata(:,1);
% pitch=imudata(:,2);
% roll=imudata(:,3);
% magx=imudata(:,4);
% magy=imudata(:,5);
% magz=imudata(:,6);
% accx=imudata(:,7);
% accy=imudata(:,8);
% accz=imudata(:,9);
% gyrox=imudata(:,10);
% gyroy=imudata(:,11);
% gyroz=imudata(:,12);
% time=[1:length(yaw)]*0.01;
% plot(x,y);
% xlim([-100,1000]);
% ylim([-200,800])
% subplot(311)
% plot(time,accx);
% subplot(312)
% plot(time,accy);
% subplot(313)
% plot(time,accz);
% figure(1)
% plot(gyroz(6800:11400));
% figure()
start=6400;
ending=7250;
% scatter3(magx(start:ending),magy(start:ending),magz(start:ending));
mag=(rotm*[magx,magy,magz]')';
magxr=mag(:,1);
magyr=mag(:,2);
% plot(magxr(start:ending),magyr(start:ending));
hold on
% scatter(mean(magxr(start:ending)),mean(magyr(start:ending)));
x_o=mean(magxr(start:ending));
y_o=mean(magyr(start:ending));
scatter(mean(magxr(start:ending)-x_o),mean(magyr(start:ending)-y_o));
plot(magxr(start:ending)-x_o,magyr(start:ending)-y_o);
grid on
%  plot(magx,magy);
hold off
XH=magxr(start:ending)-x_o-0.008;
YH=magyr(start:ending)-y_o-0.008;
% XH=magxr-x_o;
% YH=magyr-y_o;
rr=sqrt(XH.^2+YH.^2);
[M,I]=max(rr);
[M2,I2]=min(rr);
r_d=mean(rr);
% yaw_angle=[];
% for i= 1: length(XH)
%     if XH(i)<0 && YH(i)<0
%         anglee=pi-atan(YH(i)/XH(i));
%     elseif XH(i)>0 && YH(i)<0
%         anglee=atan(YH(i)/XH(i));    
%     elseif XH(i)>0 && YH(i)>0
%         anglee=2*pi-atan(YH(i)/XH(i));
%     elseif XH(i)<0 && YH(i)>0
%         anglee=pi+atan(YH(i)/XH(i));
%     elseif XH(i)==0 && YH(i)<0
%         anglee=pi/2;
%     elseif XH(i)==0 && YH(i)>0
%         anglee=3*pi/2;
%     end
%     yaw_angle=[yaw_angle;anglee];
% end

subplot(121);
scatter(XH,YH);
hold on
scatter(XH(I),YH(I),'filled');
scatter(XH(I2),YH(I2),'filled');
ra=atan2(YH(I),XH(I));
rotmm=[cos(ra),sin(ra);-sin(ra),cos(ra)];
rotmm2=[cos(-ra),sin(-ra);-sin(-ra),cos(-ra)];
grid on
grid minor
xlabel('mx[Gauss]');
ylabel('my[Gauss]');
title('Soft Iron Effect Correction(Before)')

subplot(122);
XYH=(rotmm*[XH';YH'])';
point_l=find(abs(XYH(:,1))<0.01);
yscale=r_d/mean(abs(XYH(point_l,2)));
point_l2=find(abs(XYH(:,2))<0.01);
xscale=r_d/mean(abs(XYH(point_l2,1)));
XYH_scaled=[XYH(:,1)*xscale,XYH(:,2)*yscale];
XYH_scaled_rot=(rotmm2*XYH_scaled')';
scatter(XYH_scaled_rot(:,1),XYH_scaled_rot(:,2));
grid on
grid minor
xlabel('mx[Gauss]');
ylabel('my[Gauss]');
hold on
title('Soft Iron Effect Correction(After)')

scatter(XYH_scaled_rot(I,1),XYH_scaled_rot(I,2),'filled');
scatter(XYH_scaled_rot(I2,1),XYH_scaled_rot(I2,2),'filled');
r=0.1;
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',[1,1],'linewidth',1)
grid on
hold off
%%
%????????????
clf
XYHall=(rotmm*[(magxr-x_o)';(magyr-y_o)'])';
XYH_scaledall=[XYHall(:,1)*xscale,XYHall(:,2)*yscale];
XYH_scaled_all_rot=(rotmm2*XYH_scaledall')';
YHall=XYH_scaled_all_rot(:,2);
XHall=XYH_scaled_all_rot(:,1);
plot(magxr,magyr);
hold on
plot(XHall,YHall);
clf
%%
%unlimited axis limit of Mag data
yaw_angle=atan2(YHall,XHall);
yaw_angle=yaw_angle-yaw_angle(1);
% plot(time_mag(start:ending),angle*180/pi);
plot(time_mag,yaw_angle*180/pi);

xlabel('Time[s]');
ylabel('Orientation[deg]');
% angle2=[0];
% aaaa=0;
% aaaa=yaw_angle(1);
% yaw_mag_unlimited=[aaaa];
% for i =2:length(yaw_angle)
%     differ =yaw_angle(i)-yaw_angle(i-1);
%     if differ>320*pi/180
%         differ=differ-2*pi;
%     elseif differ<-320*pi/180
%         differ=differ+2*pi;
%     end
%     yaw_mag_unlimited=[yaw_mag_unlimited;aaaa+differ];
%     aaaa=aaaa+differ;
% end
yaw_mag_unlimited=unwrap(yaw_angle);
plot(time,yaw_mag_unlimited*180/pi,'LineWidth',1.5);
hold on
yaw_mag_unlimited_low_filtered = lowpass(yaw_mag_unlimited,0.2,40);
plot(time,yaw_mag_unlimited_low_filtered*180/pi,'LineWidth',1.2);
%%
grid on
grid minor

xlabel('Time[s]');
ylabel('Orientation[deg]');
legend('original signal','low-pass filtered');
hold on
grid on

%%
%quat automatically generated from IMU
q=[quadw,quadx,quady,quadz];
eul_imu_auto=quat2eul(q,'XYZ');
yaw_imu_auto=eul_imu_auto(:,1);
hold off
plot(time,eul_imu_auto*180/pi);
legend('z','y','x')
aaab=yaw_imu_auto(1);
yaw_imu_auto_unlimited=[aaab];

for i =2:length(yaw_imu_auto)
    differ =yaw_imu_auto(i)-yaw_imu_auto(i-1);
    if differ>320*pi/180
        differ=differ-2*pi;
    elseif differ<-320*pi/180
        differ=differ+2*pi;
    end
    yaw_imu_auto_unlimited=[yaw_imu_auto_unlimited;aaab+differ];
    aaab=aaab+differ;
end
plot(time,yaw_imu_auto_unlimited*180/pi);
%%
%plot angle integrated from gyro
hold off 
clf
yaw_list_gyro=[0];%gyro yaw
w=0;
angle=0; 
% yaw_list2=[0];%gyro yaw
% w=0;
% angle=0;
% for i =2:70163
%     dt=(time(i)-time(i-1));
%     yaw_list2=[yaw_list2;angle-wz(i-1)*dt];
%     angle=angle-wz(i-1)*dt;
%     if angle>pi
%         angle=angle-2*pi;
%     elseif angle<-pi
%         angle=angle+2*pi;
%     end
% end

% wz_high_pass=highpass(wz,0.01,40);
for i =2:70163 %gyro data not limited to ±pi
    dt=(time(i)-time(i-1));
    yaw_list_gyro=[yaw_list_gyro;angle-wz(i-1)*dt];
    angle=angle-wz(i-1)*dt;
end
plot(time,yaw_mag_unlimited_low_filtered*180/pi,'LineWidth',1.5);
hold on
% hold off
plot(time,yaw_list_gyro*180/pi,'LineWidth',1.5);
grid on;
grid minor
xlabel('Time[s]');
ylabel('Orientation[deg]');
legend('Yaw calculated from magnetometer','Yaw integrated from gyro')
%%
%compensate for mag sudden drop
% yaw_mag_unlimited(768/0.025:end)=yaw_mag_unlimited(768/0.025:end)+(mean(yaw_mag_unlimited(660/0.025:740/0.025))-mean(yaw_mag_unlimited(790/0.025:900/0.025)))-10/180*pi;
% windowSize = 40; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% yaw_mag_unlimited = filter(b,a,yaw_mag_unlimited);

%%
% startfrom=9;
% yaw_list3=[0];%1 order comp filter
% w=0;
% angle=0;
hold off
clf
figure();
hold on
a=1;
yaw_list_complimentory=[0];
w=0;
angle=0;
nn=0;
bias=0;
for i =2:70163
    if i>200/0.025
        wa=var(yaw_angle(i-20:i-1));
        wb=var(yaw_list_gyro(i-20:i-1));
        a=wa/(wa+wb);
    end
    dt=(time(i)-time(i-1));
    yaw_list_complimentory=[yaw_list_complimentory;(a)*(angle-wz(i-1)*dt)+(1-a)*(yaw_mag_unlimited_low_filtered(i)+bias)];
    angle=(a)*(angle-wz(i-1)*dt)+(1-a)*(yaw_mag_unlimited_low_filtered(i));

end

bias=0;
nn=0;

plot(time,yaw_mag_unlimited*180/pi,'LineWidth',1.5);
hold on
plot(time,yaw_list_gyro*180/pi,'LineWidth',2);
plot(time,yaw_list_complimentory*180/pi,'LineWidth',2.5);
for i =2000:70163
    yaw_list_complimentory(i)=yaw_list_complimentory(i)+bias;
    if  i>295/0.025  && i<1500/0.025
        if abs(mean(yaw_list_complimentory(i-200:i))-mean(yaw_list_complimentory(i-1000:i-800)))>pi/4 && nn==0
            if mean(yaw_list_complimentory(i-200:i))-mean(yaw_list_complimentory(i-1000:i-800)) <0
                bias=-pi/2-(mean(yaw_list_complimentory(i:i+500))-mean(yaw_list_complimentory(i-500:i-200)))
                mean(yaw_list_complimentory(i-3000:i-400))*180/pi
            elseif mean(yaw_list_complimentory(i-200:i))-mean(yaw_list_complimentory(i-1000:i-800)) >0
                bias=pi/2-(mean(yaw_list_complimentory(i:i+500))-mean(yaw_list_complimentory(i-500:i-200)))
            end
        nn=2000;
        end
    end
        if nn>0
            nn=nn-1;
        end
end

grid on;
grid minor
xlabel('Time[s]');
ylabel('Orientation[deg]');
eulerfromquat=unwrap(quat2eul(quat_wrong2,'ZYX'));
yawfromquat=eulerfromquat(:,1);
plot(time,yawfromquat,'LineWidth',2);
% plot(time,yaw_list3*180/pi,'k','LineWidth',2.5);
legend('Magnetometer','IGyro','1 order complementary filter','yaw from quaternion','1 order complementary filter with adjustment');
%%