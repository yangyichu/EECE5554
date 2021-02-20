%%%%STATIC ISEC
x=rtkroverstatisecgpsdata.fieldutm_easting;
y=rtkroverstatisecgpsdata.fieldutm_northing;
% x=rtkroverstatopengpsdata.fieldLatitude;
% % y=rtkroverstatopengpsdata.fieldLongitude;

q=0*pi/180;
R=[cos(q) -sin(q);sin(q) cos(q)];
p=(R*[x,y]')';
xx=p(:,1);
yy=p(:,2);
figure(1);
xx=xx-mean(xx);
yy=yy-mean(yy);
% xx=x-mean(x);
% yy=y-mean(y);
scatter(xx,yy,'x');
% plot(xx,yy);
hold on
scatter(0,0,'filled');

grid on
grid minor
title('Static data in front of Isec');
xlabel('x[m]');
ylabel('y[m]');
xlim([-16,16]);
ylim([-16,16]);
axis equal

r=sqrt(xx.^2+yy.^2);
sigx=std(xx);
sigy=std(yy);
CEP=0.57*(sigx+sigy);
DRMS2=2*sqrt(sigx^2+sigy^2);
rectangle('Position',[-CEP,-CEP,2*CEP,2*CEP],'Curvature',[1 1],'EdgeColor','r');
text(-CEP,-CEP,'CEP');
rectangle('Position',[-DRMS2,-DRMS2,2*DRMS2,2*DRMS2],'Curvature',[1 1],'EdgeColor','g');
text(-DRMS2,-DRMS2,'2DRMS');
clf;

hist(r,200);
xlabel('x[m]');
ylabel('Samples');
qqplot(r)
% hist(yy,1000)
std(xx);
std(yy);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%STATIC OPEN
clf;
x=rtkroverstatopengpsdata.fieldutm_easting;
y=rtkroverstatopengpsdata.fieldutm_northing;
xx=x-x(1);
yy=y-y(1);
sigx=std(xx);
sigy=std(yy);
CEP=0.57*(sigx+sigy);
DRMS2=2*sqrt(sigx^2+sigy^2);
figure(1)
scatter(xx,yy);
hold on
scatter(0,0,'filled')
r=sqrt(xx.^2+yy.^2);
std(r)
grid on
grid minor
title('Static data on football field');
xlabel('x[m]');
ylabel('y[m]');
rectangle('Position',[-CEP,-CEP,2*CEP,2*CEP],'Curvature',[1 1],'EdgeColor','r');
text(-CEP,-CEP,'CEP');
rectangle('Position',[-DRMS2,-DRMS2,2*DRMS2,2*DRMS2],'Curvature',[1 1],'EdgeColor','g');
text(-DRMS2,-DRMS2,'2DRMS');
xlim([-2,2]);
ylim([-2,2]);
axis equal
% hist(xx)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
x=rtkrovermove1isecgpsdata.fieldLatitude;
y=rtkrovermove1isecgpsdata.fieldLongitude;
% x=rtkrovermove1isecgpsdata.fieldutm_easting;
% y=rtkrovermove1isecgpsdata.fieldutm_northing;
[x,y,z]=deg2utm(x,y);
ending=540;
xx=x(1:ending)-x(1);
yy=y(1:ending)-y(1);
starting=[1,201,286,485];
ending=[200,285,484,540];
std_d=[];
d1=[];
for i=1:4
    s=starting(i);
    e=ending(i);
    [p,S] = polyfit(xx(s:e),yy(s:e),1); 
    [y_fit,delta] = polyval(p,xx(s:e),S);
    eval(['subplot(22',num2str(i),');']);
    plot(xx(s:e),yy(s:e),'bx');
    hold on
    plot(xx(s:e),y_fit,'r-');
    plot(xx(s:e),y_fit+2*delta,'r--',xx(s:e),y_fit-2*delta,'r--','LineWidth',1.2);
    d=dist(xx(s:e),yy(s:e),p);
    std_d=[std_d;mean(d)];

    d1=[d1;d];
    title(['Moving trajectory in front of Isec Part ',num2str(i)]);
    xlabel('x[m]');
    ylabel('y[m]');
    axis equal
    legend('Data','Linear Fit','95% Prediction Interval')
    grid on
    grid minor
end
l=ending-starting;
moving_error_isec=sum(l*std_d,2)/ending(4);
clf;
figure();
plot(xx,yy,'ko');
hold on
for i=1:4
    s=starting(i);
    e=ending(i);
    plot(xx(s:e),yy(s:e),'o');
end
xlabel('x[m]');
ylabel('y[m]');
grid on
grid minor
title('Moving trajectory in front of Isec');
q=40*pi/180;
R=[cos(q) sin(q);-sin(q) cos(q)];
p=(R*[xx,yy]')';
% clf
% figure(1)
% scatter(p(:,1),p(:,2));
% grid on
% grid minor
% title('Moving trajectory in front of Isec');
% xlabel('x[m]');
% ylabel('y[m]');
% axis equal

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
x=rtkrovermoveopengpsdata.fieldutm_easting;
y=rtkrovermoveopengpsdata.fieldutm_northing;
xx=x-x(1);
yy=y-y(1);
figure(2)
scatter(xx,yy);
grid on
grid minor
title('Moving trajectory on football field');
xlabel('x[m]');
ylabel('y[m]');
axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=rtkrovermoveopengpsdata.fieldutm_easting;
y=rtkrovermoveopengpsdata.fieldutm_northing;
starting=[2,140,231,331];
ending=[139,230,330,426];
xx=x-x(1);
yy=y-y(1);
figure();
plot(xx,yy,'ko');
hold on
for i=1:4
    s=starting(i);
    e=ending(i);
    plot(xx(s:e),yy(s:e),'o');
end
xlabel('x[m]');
ylabel('y[m]');
grid on
grid minor
title('Moving trajectory in front of Isec');

scatter(xx,yy);
grid on
grid minor
title('Moving trajectory on football field');
xlabel('x[m]');
ylabel('y[m]');
axis equal
std_d2=[];
d2=[];
for i=1:4
    s=starting(i);
    e=ending(i);
    [p,S] = polyfit(xx(s:e),yy(s:e),1); 
    [y_fit,delta] = polyval(p,xx(s:e),S);
    eval(['subplot(22',num2str(i),');']);
    plot(xx(s:e),yy(s:e),'bx');
    hold on
    plot(xx(s:e),y_fit,'r-');
    plot(xx(s:e),y_fit+2*delta,'r--',xx(s:e),y_fit-2*delta,'r--','LineWidth',1.2);
    d=dist(xx(s:e),yy(s:e),p);
    d2=[d2;d];
    std_d2=[std_d2;mean(d)];
    title(['Moving trajectory on football field',num2str(i)]);
    xlabel('x[m]');
    ylabel('y[m]');
    axis equal
    legend('Data','Linear Fit','95% Prediction Interval')
    grid on
    grid minor
end

l=ending-starting;
moving_error_open=sum(l*std_d2,2)/ending(4);



function d=dist(m,n,p)
    k=p(1);
    b=p(2);
    d=abs(k*m-n+b)/sqrt(k^2+1);
end
    
