%%imu calibrate
% clear all
load('imugps22');
x_offset=0.046157500000005;
y_offset=0.028345833333325;
z_offset=0.365828333333334;

accx=accx-x_offset;
accy=accy-y_offset;
accz=accz-z_offset;

windowSize = 10; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
accxf = filter(b,a,accx);
accyf = filter(b,a,accy);
acczf = filter(b,a,accz);
g=[accxf,-accyf,-acczf];
g_norm=g./sqrt(accxf.^2+accyf.^2+acczf.^2);
startfrom=100;
caliend=1000;
theta=mean(-asin(g_norm(startfrom:caliend,1)));
phi=mean(asin(g_norm(startfrom:caliend,2)./cos(theta)));
phi2=mean(acos(g_norm(startfrom:caliend,3)./cos(theta)));
rotm = eul2rotm([-theta,phi2,0],'XYZ');
gg=(inv(rotm)*g')';
accxcalied=gg(:,1)-mean(gg(100:1000,1));
accycalied=gg(:,2)-mean(gg(100:1000,2));
acczcalied=gg(:,3)-mean(gg(100:1000,3));
actual_speed=[0;0;0;repelem(vel_list_gps,40,1)];
error_list=[];

for limitlimit=0.0028
    
    error=0;
vibrating_data=highpass(accz,15,40);
drivingornot=[zeros(200,1)];
for i=201:70163
    if i<400/0.025
        limit=0.0012;
    elseif i<1000/0.025 && i< i<1400/0.025
        limit=0.0025;
    else
        limit=limitlimit;
    end
    if var(vibrating_data(i-200:i))>limit
        drivingornot=[drivingornot;1];
    else
         drivingornot=[drivingornot;0];
    end
end
vel_list_imu0=[0];
forward_vel=0;
ax1=lowpass(accxcalied,0.05,40);
bias=0;
for i = 2:70163
    dt=(time(i)-time(i-1));
    ax=ax1(i);
    if drivingornot(i)==1
%         if ax<0
%             ax=ax*0.5;
%         end
        vel_list_imu0=[vel_list_imu0;forward_vel+ax*dt+bias];
        forward_vel=forward_vel+ax*dt;
    else
        vel_list_imu0=[vel_list_imu0;0];
        forward_vel=0;
    end
     if forward_vel<0
%         bias=-forward_vel;
        vel_list_imu0(1:i-1)*(max(vel_list_imu0(1:i-1))-forward_vel)/max(vel_list_imu0(1:i-1));
        forward_vel=0;
    end
    error=error+abs(forward_vel-actual_speed(i));
end
    error_list=[error_list;error];
end
% subplot(211)
plot(time,vel_list_imu0);
hold on
plot(time,drivingornot*5)
% ylim([-2,2])
% title('AccY calculated by Vx*wx')
% subplot(212)
% plot(time,lowpass(accycalied,0.5,40));
% ylim([-2,2])
% title('AccY measured')

%%
%%
vel_list_gps=[0];
forward_vel=0;
for i=2:1754
    velx=utm_x(i)-utm_x(i-1);
    vely=utm_y(i)-utm_y(i-1);
    vel_list_gps=[vel_list_gps;sqrt(velx^2+vely^2)];
end
hold on
driving_time=[0;0;0;repmat(logical(vel_list_gps),40,1)];
driving_time_index=find(driving_time>0);
not_driving_time_index=find(driving_time==0);
driving_z=acczcalied(driving_time_index);
not_driving_z=acczcalied(not_driving_time_index);
var_list1=[];
var_list2=[];

plot(time_gps./1e9,vel_list_gps);
% plot(time,accxcalied_lowpass);
% legend('Velocity integrated from accelerometer','Velocity calculated from GPS');
%%
hold on
pitch=0;
forward_vel=0;
vel_list_imu=[0];
alpha=0;
ax=0;
accxcalied_lowpass=lowpass(accxcalied,0.01,40);
nn=0;
for i=8000:65000
    if var(acczcalied(i-100:i))<0.006 && nn==0
        scatter(time(i),0,'rx');
        nn=200;
    end
    if nn>0
        nn=nn-1;
    end
end
xlabel('time[s]');
ylabel('velocity[m/s]');
nn=0;

for i=2:70163
    ax=accxcalied_lowpass(i-1);
%     end
%     ax=alpha*ax+(1-alpha)*acce;
    dt=(time(i)-time(i-1));
    vel_list_imu=[vel_list_imu;forward_vel+ax*dt];
    forward_vel=forward_vel+ax*dt;
    if forward_vel<0
        forward_vel=0;
    end
    if i>8000
%         if var(acczcalied(i-20:i))<0.001 && nn==0
        if var(acczcalied(i-20:i))<0.001 && nn==0    
        scatter(time(i),0,'rx');
        forward_vel=0;
        nn=2000;
        end
        if nn>0
        nn=nn-1;
        end
    end
end
hold on
plot(time,vel_list_imu,'LineWidth',1.5);
xlabel('time[s]');
ylabel('velocity[m/s]');
hold on

%%
aa=0.1;
vel_list_comp=[0];
forward_vel=0;
for i=2:70163
    if mod(i,40)==1
        ax=accxcalied(i-1);
        vel_list_comp=[vel_list_comp;aa*(forward_vel+ax*0.025)+(1-aa)*vel_list_gps(round(i/40))];
        forward_vel=aa*(forward_vel+ax*0.025)+(1-aa)*vel_list_gps(round(i/40));

    else
        ax=accxcalied(i-1);
        vel_list_comp=[vel_list_comp;forward_vel+ax*0.025];
        forward_vel=forward_vel+ax*0.025;
    end
end
hold off
plot(time,vel_list_comp);
 
hold off;
plot(utm_x-utm_x(1),utm_y-utm_y(1));

hold on
position_list=[0,0];
pos=[0,0];
yaw_list4=smooth(yaw_list3,400);
vel_list_gps_ex=[0;0;0;repelem(vel_list_gps,40)];
for i=220/0.025:70163
    RR=eul2rotm([0,0,-yaw_list3(i)+140/180*pi],'XYZ');
    v=inv(RR)*[vel_list_imu0(i);0;0];
%     v=inv(RR)*[vel_list_gps_ex(i);0;0];
%     v=inv(RR)*[vel_list_gps(round(i/41+1));0;0];
    current_vel=[v(1),v(2)];
    dt=(time(i)-time(i-1));
    position_list=[position_list;pos+current_vel*dt];
    pos=pos+current_vel*dt;
end
plot(position_list(:,1),position_list(:,2),'LineWidth',1.5)
grid minor
%%
% 
% subplot(311)
% plot(wx(startfrom:caliend));
% subplot(312)
% plot(wy(startfrom:caliend));
% subplot(313)
% plot(wz(startfrom:caliend));


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
% 
% startfrom=550;
% caliend=1200;
% startfrom=1;
% caliend=70171;
% startfrom=6400;
% caliend=9000;
% plot(accz(startfrom:end))
% quiver3(zeros(1192,1),zeros(1192,1),zeros(1192,1),accx(startfrom:caliend),accy(startfrom:caliend),accz(startfrom:caliend));
% gM=[accx(startfrom:caliend),accy(startfrom:caliend),accz(startfrom:caliend),ones(caliend-startfrom+1,1)];
% g=[zeros(caliend-startfrom+1,1),zeros(caliend-startfrom+1,1),ones(caliend-startfrom+1,1)*9.8];
% R=inv(gM'*gM)*gM'*g;
% acc=[accx,accy,accz,ones(length(accx),1)]*R;

% subplot(311)
% plot(accx(startfrom:caliend)-0.4544);
% subplot(312)
% plot(accy(startfrom:caliend)-0.3842);
% subplot(313)
% plot(accz(startfrom:caliend)-0.2869);
%%
% windowSize = 10; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% accxf = filter(b,a,accx);
% accyf = filter(b,a,accy);
% acczf = filter(b,a,accz);
% g=[accxf,-accyf,-acczf];
% g_norm=g./sqrt(accxf.^2+accyf.^2+acczf.^2);
% startfrom=1;
% caliend=1000;
% theta=mean(-asin(g_norm(startfrom:caliend,1)));
% phi=mean(asin(g_norm(startfrom:caliend,2)./cos(theta)));
% phi2=mean(acos(g_norm(startfrom:caliend,3)./cos(theta)));

% q = eul2quat([theta,phi,0],'XYZ');
% rotm = eul2rotm([theta,phi,0],'XYZ')
% for i=startfrom:caliend
%     gm=[accx(i);accy(i);accz(i)]
%     inv(rotm)*gm
% end
%%
% windowSize = 10; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% y = filter(b,a,accx);
% plot(accx)
% hold on
% plot(y) 
%%
% fn=16000;
% ap=0.1;
% as=60;
% wp=2000;
% ws=5000; %???????
% wpp=wp/(fn/2);wss=ws/(fn/2);  %???;
% [n,wn]=buttord(wpp,wss,ap,as); %????????
% [b,a]=butter(n,wn); %??N???????????????????????????b?a?
% freqz(b,a,512,fn);
%%
% plot(accx(startfrom:caliend)-0.4544);
% subplot(312)
% plot(accy(startfrom:caliend)-0.3842);
% subplot(313)
% plot(accz(startfrom:caliend)-0.2869);
% pitch_list=[0];
% angle=0;
% for i =2:70163
%     dt=(time(i)-time(i-1));
%     pitch_list=[pitch_list;angle-wy(i-1)*dt];
%     angle=angle-wy(i-1)*dt;
%     if angle>pi
%         angle=-pi;
%     elseif angle<-pi
%         angle=pi;
%     end
% end
% plot(pitch_list(1:1000).*180/pi);

%%
%kalman
% hold on
% pitch=0;
% forward_vel=0;
% vel_list_imu=[0];
% alpha=0.2;
% ax=0;
% X=[0;0];
% P=[0.05 0;0 0.05];
% Q=[0.5 0;0 0.5];
% H=[0 1];
% R=0.0142;
% X_list=[X'];
% x_bias = 0;
% for i=2:70163
%     dt=time(i)-time(i-1);
%     dt=0.025;
%     F=[1 dt;0 1];
%     X_=F*X;
%     P_=F*P*F'+Q;
%     K=P_*H'/(H*P_*H'+R);
%     X=X_+K*(accxcalied(i-1)-H*X_);
%     X_list=[X_list;X'];
%     P=(eye(2)-K*H)*P_;
% end
% plot(time,X_list(:,1));
% forward_vel=0;
% vel_list_imu2=[0];
% for i=2:70163
%     
% %     ax=(accxf(i-1)-9.8*sin(pitch_list(i-1)))/cos(pitch_list(i-1));
%     if abs(accxcalied(i-1))<0.1
%         acce=0;
%     elseif abs(accxcalied(i-1))>5
%         acce=accxcalied(i-1)/accxcalied(i-1)*5;  
%     else
%         acce=accxcalied(i-1);
%     end
%     ax=alpha*ax+(1-alpha)*acce;
%     dt=(time(i)-time(i-1));
%     if mod(i,40)==1
%         vel_list_imu2=[vel_list_imu2;alpha*(forward_vel+ax*dt)+(1-alpha)*vel_list_gps(round(i/40))];
%         forward_vel=alpha*(forward_vel+ax*dt)+(1-alpha)*vel_list_gps(round(i/40));
%     else
%         vel_list_imu2=[vel_list_imu2;forward_vel+ax*dt];
%         forward_vel=forward_vel+ax*dt;
%     end
%     if i>1000 && i<70100 && mean(accxcalied(i-10:3:i-1))<-0.3 && abs(mean(accxcalied(i:3:i+50)))<0.05
%         forward_vel=0;
%     end
% 
%     if forward_vel<0
%          forward_vel=0;
%     end
% end
% plot(time,vel_list_imu2);
