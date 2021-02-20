% imudata=[xdown;xup;ydown;yup;zdown;zup];
% 

% yaw=imudata(:,1);
% pitch=imudata(:,2);
% roll=imudata(:,3);
% magx=imudata(:,1);
% magy=imudata(:,2);
% magz=imudata(:,3);
% accx=imudata(:,4);
% accy=imudata(:,5);
% accz=imudata(:,6);
gyrox=gyrozup(:,10);
gyroy=gyrozup(:,11);
gyroz=gyrozup(:,12);
plot(gyrox);
hold on
plot(gyroy);
plot(gyroz)