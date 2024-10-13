function ref= ReferenceGenerator_DLC(Ts,plotter,speed,repeated)

%% %%%% NOTE: THIS FILE IS FOR DLC ONLY
load("ReferenceRaw_DLC_100.mat"); % Change the mat file name to whichever reference you generated

if speed==100
    if repeated
        load('lat_error_dlc100_repeated.mat');
    else
        load('lat_error_dlc100.mat');
    end
else
    load('lat_error_dlc70.mat');
end
Ts_data= 1/50; % Preferred sample time for reference

x_ref= out.Reference.signal_266__X_Position_DriverModelTarget_global__m_.Data(:,1);
y_ref= out.Reference.signal_267__Y_Position_DriverModelTarget_global__m_.Data(:,1);

v_act= 0.277778*speed;

if repeated
        x_ref=[x_ref(1:end-1);x_ref(end)+x_ref];
        x_ref=[x_ref(1:end-1);x_ref(end)+x_ref];
        y_ref=[y_ref(1:end-1);-(y_ref(end)+y_ref)];
        y_ref=[y_ref(1:end-1);y_ref(end)+y_ref];
end

rows=length(x_ref);
t_ref(1,1)=0;

% Calculating desired time of arrival for each waypoint. 
for i=2:rows
    dist= sqrt(((x_ref(i)-x_ref(i-1))^2)+((y_ref(i)-y_ref(i-1))^2));
    t_ref(i,1)= t_ref(i-1)+dist/v_act;
end

trajectory = waypointTrajectory([x_ref, y_ref, zeros(rows,1)], ...
    'TimeOfArrival',t_ref, ...
    'SampleRate',1/Ts_data);

count=1;

while ~isDone(trajectory)
   %[currentPosition,orientationLog(count)] = trajectory();
   %plot(currentPosition(1),currentPosition(2),'bo')
   [pos(count,:),orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();
   time(count)= (count-1)*Ts_data;
    %pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count + 1;
end
t_ref=time';


 size= length(pos);
 x_ref=pos(:,1);
 y_ref=pos(:,2);
 v_ref=v_act*ones(size,1);
 a_ref= zeros(size,1);
% [row2,~]=size(trajectory.GroundSpeed);
% ground_speed=trajectory.GroundSpeed;
% ground_speed= resample(ground_speed,row,row2);

for i=1:size
   euler(:,i)= eulerd(orient(i),'ZYX','frame');
   yaw(i)= deg2rad(euler(1,i));
end
curvature_ref= angVel(:,3)./vel(:,1);
theta_ref=yaw';
Tsim= t_ref(end);
t_vec= (0:Ts:Tsim)';
    
    ref.x_ref=makima(t_ref,x_ref,t_vec);
    ref.y_ref=makima(t_ref,y_ref,t_vec);
    ref.psi_ref=makima(t_ref,theta_ref,t_vec);
    ref.v_ref= makima(t_ref,v_ref,t_vec);
    ref.curvature_ref=makima(t_ref,curvature_ref,t_vec);
    ref.a_ref=makima(t_ref,a_ref,t_vec);
    ref.t_ref=t_vec;
    ref.lat_error=lat_error;
   
    
    if plotter
        figure(1)
        nexttile
        plot(x_ref,y_ref);
        hold on
        plot(ref.x_ref,ref.y_ref);
        hold off
        title("Reference Path")
        nexttile
        plot(t_ref,a_ref);
        hold on
        plot(ref.t_ref,ref.a_ref);
        hold off
        title("Reference Acceleration")
        nexttile
        plot(t_ref,curvature_ref);
        hold on
        plot(ref.t_ref,ref.curvature_ref);
        hold off
        title("Reference Curvature")
        nexttile
        plot(t_ref,v_ref);
        hold on
        plot(ref.t_ref,ref.v_ref);
        hold off
        title("Reference Speed")
        nexttile
        plot(t_ref,theta_ref);
        hold on
        plot(ref.t_ref,ref.psi_ref);
        hold off
        title("Reference Yaw")
    end
    ref.x_ref(end+1:end+200)= ref.x_ref(end);
    ref.y_ref(end+1:end+200)= ref.y_ref(end);
    ref.psi_ref(end+1:end+200)= ref.psi_ref(end);
    ref.v_ref(end+1:end+200)= ref.v_ref(end);
    ref.curvature_ref(end+1:end+200)= ref.curvature_ref(end);
    ref.a_ref(end+1:end+200)= 0;
end
