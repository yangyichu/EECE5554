clear all
base_footprint=[0,0,0.33];
cam_array_base_link= [1.5,0.66,1.58]+base_footprint;
vlp16_port_base_link=[-0.1,0,0.0623]+cam_array_base_link;
vlp16_port=[0,0,0.0377]+vlp16_port_base_link;
vlp16_starboard_base_link=[-0.1,-1.32,0.0623]+cam_array_base_link;
vlp16_starboard_port=[0,0,0.0377]+vlp16_starboard_base_link;
tformcam=affine3d([eye(3,4);vlp16_starboard_port-vlp16_port,1]);

%flags
video_record_flag=0;
pcshow_flag=1;

%params
yaw_compensation_alpha=0.8;
pos_compensation_alpha=0.4;
accuracy=[1e-6,1e-6];
start=1;
step=4;
gridSize=0.25;
mergeSize = 0.25;
trajectory_offset=2.5;
distThreshold = 1.2;
camera_offset=[0;0;trajectory_offset];
colormap_type='bone';

%loading data
[gps_time,gps_lat,gps_lon,utm_x,utm_y,utmzone,position_gps]=load_gps();
[imu_time,yaw_from_imu]=load_imu();
[namelist_pcd1,namelist_pcd2,time_lidar_list]=load_lidar_info();
rmse_list=[];

%video record
video_name='my_demo_vid.avi';
if video_record_flag>0
    vid_moving_scenes = VideoWriter(video_name,'Motion JPEG AVI');
    open(vid_moving_scenes);
end

initial_position=[0;0;0;1];
position_list=[0;0;0;1];
yaw_list=[];
eul_sum_from_frames=[0,0,0];
frame_num=0;
previous_yaw=0;
prev_gps_positon=[0,0];
vel_list=[0,0];
offset=-1.8638;
Rz = [cos(offset) -sin(offset); sin(offset) cos(offset)];
position_gps_rot=(Rz*position_gps')'-repmat([6,0],length(position_gps),1);
eul_list=[0];
tformcamlist=[];

for i =start:step:length(namelist_pcd2)
    time_lidar=time_lidar_list(i);
    %get index in imu_data at the same time
    imu_index=index_mapping(imu_time,time_lidar);
    gps_index=index_mapping(gps_time,time_lidar);
    time_lidar=time_lidar-time_lidar_list(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if i==start
        filename1=[namelist_pcd1(i).folder,'\',namelist_pcd1(i).name];
        ptCloud_current1 = pcread(filename1);
        ptCloud_prev=reconstruct(ptCloud_current1);
        
        ptCloud_prev_downsampled = pcdownsample(ptCloud_prev, 'gridAverage', gridSize);
        tform=affine3d(eye(4));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    elseif i==start+step
        %only using one lidar data set, codes for merging two data sets
        %has been deleted
        filename1=[namelist_pcd1(i).folder,'\',namelist_pcd1(i).name];
        ptCloud_current1 = undistort_lidar(pcread(filename1),vel_list(end,:));
        
        ptCloud_current=reconstruct(ptCloud_current1);
        [g,ptCloud_current,~,~]=extract_planes(ptCloud_current);
        ptCloud_current_downsampled = pcdownsample(ptCloud_current, 'gridAverage', gridSize);
        [tform,movingReg,rmse] = pcregistericp(ptCloud_current_downsampled, ptCloud_prev_downsampled, 'Metric','pointTopoint','Extrapolate', true,'MaxIterations',100,'Tolerance',accuracy,'InlierRatio',0.6,'InitialTransform',tform);
        rmse_list=[rmse_list;rmse];
        ptCloud_prev_downsampled=ptCloud_current_downsampled;
        
        % Visualize the input images.
        fig=figure(1);
        set(gcf,'Position',get(0,'ScreenSize'));
        
        %ignore roll and pitch
        eul_from_frames=rotm2eul(tform.T(1:3,1:3)');
        eul_sum_from_frames=eul_sum_from_frames+eul_from_frames;
        if((eul_sum_from_frames(1)-previous_yaw)>pi)
            yaw_cur=eul_sum_from_frames(1)-2*pi;
        else
            yaw_cur=eul_sum_from_frames(1);
        end
        previous_yaw=yaw_cur;
        eul_list=[eul_list;yaw_cur];
        accumTform = tform;
        yaw_comped=(1-yaw_compensation_alpha)*yaw_cur+yaw_compensation_alpha*yaw_from_imu(imu_index);
        yaw_list=[yaw_list;yaw_comped];
        position=accumTform.T'*initial_position;
        
        if (~isequal(gps_index,[])&&isequal(position_gps_rot(gps_index,:),prev_gps_positon))
            position_comped=[position(1:2);initial_position(3);1]*(1-pos_compensation_alpha)+[position_gps_rot(gps_index,:),initial_position(3),1]'*pos_compensation_alpha;
            prev_gps_positon=position_gps_rot(gps_index,:);
        else
            position_comped=[position(1:2);initial_position(3);1];
        end
        
        position_list=[position_list,position_comped];
        
        vel=(position_list(end,:)-position_list(end-1,:))/(time_lidar_list(i)-time_lidar_list(i-step));
        vel_list=[vel_list;vel(1:2)];
        
        accumTform_comped=affine3d([eul2rotm([yaw_comped,0,0])',[0;0;0];position_comped']);
        accumTform=accumTform_comped;
        
        ptCloudAligned = pctransform(ptCloud_current,accumTform);
        ptCloudScene = pcmerge(ptCloud_prev, ptCloudAligned, mergeSize);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif i>start+step
        
        %read pcd file left and right then undistort them
        filename1=[namelist_pcd1(i).folder,'\',namelist_pcd1(i).name];
        ptCloud_current1 = undistort_lidar(pcread(filename1),vel_list(end,:));
        ptCloud_current_1=reconstruct(ptCloud_current1);
        
        %Euclidean Clustering
        [g,ptCloud_current,inlier_g,outlier_g]=extract_planes(ptCloud_current_1);
        ptCloudWithoutGround = select(ptCloud_current_1,outlier_g,'OutputSize','full');
        [buildings,labels,numClusters]=getbuildings(ptCloudWithoutGround,distThreshold);
        [~,ptCloudWithoutGroundandCar,inlier_car,outlier_car]=extract_car(ptCloud_current_1);
        car_label=numClusters+2;
        g_label=numClusters+1;
        labels(inlier_car) = car_label;
        labels(inlier_g) = g_label;
        labelColorIndex = labels;
        
        ptCloud_current_downsampled = pcdownsample(ptCloud_current, 'gridAverage', gridSize);
        % Apply ICP registration.
        %         tic
        [tform,movingReg,rmse] = pcregisterndt(ptCloud_current_downsampled, ptCloud_prev_downsampled,1,'OutlierRatio',0.6,'MaxIterations',100,'Tolerance',accuracy,'InitialTransform',tform);
        %         toc
        
        %       get euler angle from two frames prev and current
        rmse_list=[rmse_list;rmse];
   
        % 1-order compensation filter yaw
        % ignore roll and pitch
        eul_from_frames=rotm2eul(tform.T(1:3,1:3)');
        eul_sum_from_frames=eul_sum_from_frames+eul_from_frames;
        if((eul_sum_from_frames(1)-previous_yaw)>pi/2)
            yaw_cur=eul_sum_from_frames(1)-2*pi;
        else
            yaw_cur=eul_sum_from_frames(1);
        end
        previous_yaw=yaw_cur;
        accumTform = affine3d(tform.T * accumTform.T);
        eul_list=[eul_list;yaw_cur];
        yaw_comped=(1-yaw_compensation_alpha)*yaw_cur+yaw_compensation_alpha*yaw_from_imu(imu_index);
        yaw_list=[yaw_list;yaw_comped];
        % 1-order compensation filter position
        position=accumTform.T'*initial_position;
        
        if ~isequal(position_gps_rot(gps_index,:),prev_gps_positon)
            position_comped=[position(1:2);initial_position(3);1]*(1-pos_compensation_alpha)+[position_gps_rot(gps_index,:),initial_position(3),1]'*pos_compensation_alpha;
            prev_gps_positon=position_gps_rot(gps_index,:);
        else
            position_comped=[position(1:2);initial_position(3);1];
        end
        position_list=[position_list,position_comped];
        vel=(position_list(end,:)-position_list(end-1,:))/(time_lidar_list(i)-time_lidar_list(i-step));
        vel_list=[vel_list;vel(1:2)];
        accumTform_comped=affine3d([eul2rotm([yaw_comped,0,0])',[0;0;0];position_comped']);
        accumTform=accumTform_comped;
        
        %Update Map every 5 frames
        if mod(frame_num,5)==0
            ptCloudAligned = pctransform(buildings, accumTform);
            ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
        end
        % Update the world scene.
        
        if pcshow_flag==1
            cla;
            if exist('antn')
                delete(antn);
            end
            str=['Time=',num2str(time_lidar),'s'];
            antn=annotation('textbox', [0.1 0.7 0.1 0.2], 'string', str,'Color','w','FontSize',20,'LineStyle','None');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            subplot(221)
            pcshow(ptCloud_current_1.Location,labelColorIndex);
            colormap(gca,[hsv(numClusters);[0 0 0];[1 0.5 1]]);           
            xlim([-30,+30]);
            ylim([-30,+30]);
            zlim([-2,15]);
            hold all
            for nc = 1:numClusters
                location=double(ptCloud_current_1.Location(labels==nc,:));
                if length(location)>100
                    x=location(:,1);
                    y=location(:,2);
                    z=location(:,3);                 
                    [~,cornerpoints,volume,maxlength,l1,l2,l3] = myminboundbox(x,y,z);
                    [lpca1,lpca2]=getpcalength([x,y]);
                    if volume<100 && maxlength<10 && l3<3 && min(lpca1,lpca2)<4 && min(lpca1,lpca2)>1 && max(lpca1,lpca2)<6 && max(lpca1,lpca2)>3
                        plotminbox(cornerpoints,rand(1,3)); 
                    end
                end
            end
            carpoints=[3,1.5,2;3,-1.5,2;3,-1.5,0;3,1.5,0;-3,1.5,2;-3,-1.5,2;-3,-1.5,0;-3,1.5,0;];
            plotminbox(carpoints,'white');
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            subplot(222)
            hold on
            ptCloudScene=pcdownsample(ptCloudScene, 'gridAverage', 0.4);
            [xlimit,ylimit]=getplotlim(ptCloudScene);
            pcshow(ptCloudScene, 'VerticalAxis','Z', 'VerticalAxisDir', 'Up');
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            zlim([-2,15]);
            xlim(xlimit);
            ylim(ylimit);
            eval(['colormap(gca,',colormap_type,');']);
            colorbar
            caxis([-5 8]);
            view(2);
            plot3(position_list(1,:),position_list(2,:),position_list(3,:)+trajectory_offset,'Color','cyan','LineWidth',2.5);
            R = [0,0,1;0,-1,0;1,0,0];
            cam = plotCamera('Location',position_comped(1:3)+camera_offset,'Orientation',eul2rotm([0,0,-yaw_comped])*R,'Size',2);
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            subplot(223)
            plot(time_lidar_list(2:step:i)-time_lidar_list(1),yaw_list,'LineWidth',1.5);
            hold on
            plot(imu_time(1:imu_index)-imu_time(1),yaw_from_imu(1:imu_index),'LineWidth',1.5);
            set(gca, 'XColor', 'w');
            set(gca, 'YColor', 'w');
            set(gca, 'color', 'k');
            xlabel("Time[s]",'Color','w');
            ylabel("Yaw[rad]",'Color','w');
            legend(["comped Yaw","IMU Yaw"],'TextColor','w');
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            subplot(224)
            plot(position_list(1,3:frame_num+1),position_list(2,3:frame_num+1),'LineWidth',2);
            hold on;
            plot(position_gps_rot(1:gps_index,1),position_gps_rot(1:gps_index,2),'LineWidth',2);
            set(gca, 'XColor', 'w');
            set(gca, 'YColor', 'w');
            set(gca, 'color', 'k');
            xlabel("x[m]",'Color','w');
            ylabel("y[m]",'Color','w');
            legend(["comped Position","GPS Position"],'TextColor','w');
            hold off
            drawnow;
        else
            if(mod(frame_num,10)==0)
                subplot(121)
                plot(time_lidar_list(2:step:i)-time_lidar_list(1),yaw_list,'LineWidth',1.5);
                hold on
                plot(imu_time(1:imu_index)-imu_time(1),yaw_from_imu(1:imu_index),'LineWidth',1.5);
                set(gca, 'XColor', 'w');
                set(gca, 'YColor', 'w');
                xlabel("Time[s]");
                ylabel("Yaw[rad]");
                legend(["comped Yaw","IMU Yaw"],'TextColor','w');
                hold off
                subplot(122)
                plot(time_lidar_list(2:step:i)-time_lidar_list(1),rmse_list);
                set(gca, 'XColor', 'w');
                set(gca, 'YColor', 'w');
                set(gca, 'color', 'k');
                xlabel("Time[s]",'Color','w');
                ylabel("RMSE[m]",'Color','w');
                plot(position_list(1,3:frame_num+1),position_list(2,3:frame_num+1),'LineWidth',2);
                hold on;
                plot(position_gps_rot(1:gps_index,1),position_gps_rot(1:gps_index,2),'LineWidth',2);
                set(gca, 'XColor', 'w');
                set(gca, 'YColor', 'w');
                xlabel("x[m]");
                ylabel("y[m]");
                axis equal
                legend(["comped Position","GPS Position"],'TextColor','w');
                hold off
                drawnow;
            end
            
        end
        ptCloud_prev_downsampled=ptCloud_current_downsampled;
        if video_record_flag>0
            frame=getframe(fig);
            writeVideo(vid_moving_scenes,frame);
        end
    end
    frame_num=frame_num+1;
    
end
if video_record_flag>0
    close(vid_moving_scenes);
end
%main code ends



function [gps_time,gps_lat,gps_lon,utm_x,utm_y,utmzone,position_gps]=load_gps()
    load('../../data/gpsdata');
    gps_time=gps(:,1)*1e-9;
    gps_lat=gps(:,7);
    gps_lon=gps(:,8);
    [utm_x,utm_y,utmzone] = deg2utm(gps_lat,gps_lon);
    position_gps=[utm_x,utm_y]-[utm_x(1),utm_y(1)];
end


function [imu_time,yaw_from_imu]=load_imu()
    load('../../data/imudata');
    imu_time=imudata(:,1)*1e-9;
    quad.x=imudata(:,5);
    quad.y=imudata(:,6);
    quad.z=imudata(:,7);
    quad.w=imudata(:,8);
    eul_from_imu=quat2eul([quad.x,quad.y,quad.z,quad.w]);
    yaw_from_imu=-unwrap(eul_from_imu(:,3));
    yaw_from_imu=yaw_from_imu-ones(length(yaw_from_imu),1)*mean(yaw_from_imu(1:2500));
    bias=0.03;
    hold on
    for i=1:length(yaw_from_imu)
        yaw_from_imu(i)=yaw_from_imu(i)-(i*-1.38420813571845e-05)-bias;
    end
end

function time_index=index_mapping(time_list,time)
    t_diff=ones(length(time_list),1)*time-time_list;
    [~, time_index] = min(t_diff(t_diff>0));
end

function [namelist_pcd1,namelist_pcd2,time_lidar_list]=load_lidar_info()
    namelist_pcd1 = dir('..\..\data\pcd1\*.pcd');
    namelist_pcd2 = dir('..\..\data\pcd2\*.pcd');
    time_lidar_list=[];
    for i=1:length(namelist_pcd1)
        time_lidar_list=[time_lidar_list;str2double(namelist_pcd1(i).name(1:end-4))];
    end
end

function [ground,remainPtCloud,inlierIndices,outlierIndices]=extract_planes(ptCloud)
    maxDistance = 0.15;
    referenceVector = [0,0,1];
    maxAngularDistance = 0.5;
    [~,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
        maxDistance,referenceVector,maxAngularDistance);
    ground = select(ptCloud,inlierIndices);
    remainPtCloud = select(ptCloud,outlierIndices);
end

function [car,remainPtCloud,inlierIndices,outlierIndices]=extract_car(ptCloud)
    car_width = 3;
    car_length = 6;
    outlierIndices=[1:length(ptCloud.Location)];
    inlierIndices = find(abs(ptCloud.Location(:,1))<0.5*car_length & abs(ptCloud.Location(:,2))<0.5*car_width );
    outlierIndices(inlierIndices)=[];
    car = select(ptCloud,inlierIndices,'OutputSize','full');
    remainPtCloud = select(ptCloud,outlierIndices,'OutputSize','full');
end

function [rotmat,cornerpoints,volume,maxlength,l1,l2,l3] = myminboundbox(x,y,z)
    rotmat=eye(3);
    cornerpoints=[max(x),max(y),max(z);max(x),min(y),max(z);max(x),min(y),min(z);max(x),max(y),min(z);min(x),max(y),max(z);min(x),min(y),max(z);min(x),min(y),min(z);min(x),max(y),min(z);];
    l1=abs(max(x)-min(x));
    l2=abs(max(y)-min(y));
    l3=abs(max(z)-min(z));
    volume=l1*l2*l3;
    maxlength=max([l1,l2,l3]);
end

function [xlimit,ylimit]=getplotlim(ptCloud)
    x=ptCloud.Location(:,1);
    y=ptCloud.Location(:,2);
    xlimit=[min(x),max(x)];
    ylimit=[min(y),max(y)];
end

function newptcloud=reconstruct(ptcloud)
    height_bias=1;
    location=ptcloud.Location+repmat([0,0,height_bias],length(ptcloud.Location),1);
    n=find(abs(location(:,2))>40 | abs(location(:,1))>40);
    location(n,:)=[];
    m=find(location(:,3)<0);
    location(m,3)=location(m,3)*0;
    newptcloud=pointCloud(location);
end

function [buildings,labels,numClusters]=getbuildings(ptcloud,distThreshold)
    [labels,numClusters] = pcsegdist(ptcloud,distThreshold);
    ind=[1:length(labels)]';
    labelcount=accumarray(labels+1,1);
    uni=unique(labels);
    inlinerlabels=uni(labelcount>200);
    indices=[];
    for n=1:length(inlinerlabels)
        indices=[indices;ind(labels==inlinerlabels(n))];
    end
    buildings = select(ptcloud,indices);
end

function newptcloud=undistort_lidar(ptCloud_current,vel)
    dt=0.1;
    x=ptCloud_current.Location(:,1);
    y=ptCloud_current.Location(:,2);
    z=ptCloud_current.Location(:,3);
    rot0=atan2(x(1),y(1));
    rot_=0;
    for jj =1:length(x)
        rot=atan2(x(jj),y(jj));
        if rot-rot_<-pi
            rot=rot+2*pi;
        end
        rot_=rot;
        rot_time=(rot-rot0)/2/pi*dt;
        x(jj)=x(jj)+vel(1)*rot_time;
        y(jj)=y(jj)+vel(2)*rot_time;
    end
    newptcloud=pointCloud([x,y,z]);
end

function [lpca1,lpca2]=getpcalength(pos)
    x=pos';
    X=x*x';
    [U,V]=eig(X);
    u1=U(:,1);
    u2=U(:,2);
    e1=0; e2=0;
    for i=1:length(x)
        e1=e1+norm(x(:,i))^2-(x(:,i)'*u1)^2;
        e2=e2+norm(x(:,i))^2-(x(:,i)'*u2)^2;
    end
    proj=(x'*u1);
    lpca1=abs(max(proj)-min(proj));
    proj2=(x'*u1);
    lpca2=abs(max(proj2)-min(proj2));
end