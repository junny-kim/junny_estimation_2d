clear all
close all
clc



%% options
force_option=1;
leg_num = 2;
for_num = 10;
dt=0.05;
%% initialization


robot = robot_model(leg_num,dt);

% addpath("I:\Dropbox\study\matlab_jh\manifold_functions")
addpath("basic")
addpath("basic/snopt")
addpath("basic/manifold_functions")
% robot_1 = robot_model(leg_num);

q_log=[];
u_log=[];
robot.q = [-3*pi/4;-pi/2;-3*pi/4;-pi/2];
% robot.qdot = [1.0;0;0;0];
robot.qddot = [0.0;0;0;0];
robot.u = [0;0;0;0];
% initial_f=[0;0;-robot.body_mass*robot.g(3)/2;0;0;-robot.body_mass*robot.g(3)/2];
initial_f=[0;0;80*4.905+50*sin(1.57+1/3);100;0;40*4.905];
robot.f = initial_f;


initial_p=[0;0;0];
initial_v=[0;0;0];
robot.x = initial_p;
robot.v = initial_v;
initial_theta = [0;pi/6;0];
R_initial=EUL_ZYX_to_R_bw(initial_theta);

robot.theta = initial_theta;
robot.R = R_initial;
robot.x_foot=forward_kinematics_2d_robot(robot,[-3*pi/4;-pi/2;-3*pi/4;-pi/2]);
front_foot_position = robot.x + robot.R'*robot.x_foot(1:3);
rear_foot_position = robot.x + robot.R'*robot.x_foot(4:6);
robot.J = jacobian_2d_robot(robot,robot.q);
cost_MAP=0;
out=[];

fig = figure(1);
draw_robot(fig,robot)

y_out=[];
t_out=[];


%% simulate
for i=1:for_num+1
    % randn(1)
    %     robot.f = [10*sin(i/10)+randn(1);0;20*4.905+80*sin(i/10)+randn(1);10*sin(i/10)+randn(1);0;20*4.905+20*sin(i/10)+randn(1)];
    %     robot.f = [50;0;18*4.905;50;0;8*4.905];
    robot.f = [0;0;80*4.905+50*sin(1.57+i/3);100;0;40*4.905];
    temp_f = robot.f;
    
    
    robot.f = robot.f + [100*sin(i/3);0;100*sin(i/3+1.57);30*sin(i/3);0;100*sin(i/3+1.57)];
    
    
    %     check
    tspan = linspace(0,dt,11);
    state_init = [robot.theta;robot.w;robot.x;robot.v];
    [t,y]=ode45(@(t,y) dynamics_2d_robot(t,y,robot),tspan,state_init);
    
    if i==for_num
        t_out=[t_out; dt*(i-1)+t((1:end),:)];
        y_out=[y_out; y((1:end),:)];
        
    else
        t_out=[t_out; dt*(i-1)+t((1:end-1),:)];
        y_out=[y_out; y((1:end-1),:)];
    end
    
    
    
    
    
    
    %     CHECK(:,i)=IMU((4:6),i)-robot.R'*((TRUE_F(1:3,i)+TRUE_F(4:6,i))./robot.body_mass+2.*robot.g);
    %     ESTIMATE_F((1:3),i)=J_ENC((1:3),(1:2))*inv(J_ENC((1:3),(1:2))'*J_ENC((1:3),(1:2)))*TORQUE((1:2),i);
    %     ESTIMATE_F((4:6),i)=J_ENC((1:3),(3:4))*inv(J_ENC((1:3),(3:4))'*J_ENC((1:3),(3:4)))*TORQUE((3:4),i);
    %
    %
    
    %%%      update
    robot.theta=y(end,(1:3))';
    robot.R=EUL_ZYX_to_R_bw(robot.theta);
    robot.w=y(end,(4:6))';
    robot.x=y(end,(7:9))';
    robot.v=y(end,(10:12))';
    %     robot.x_foot(1:3) = robot.R'*(front_foot_position-robot.x);
    %     robot.x_foot(4:6) = robot.R'*(rear_foot_position-robot.x);
    robot.x_foot = forward_kinematics_2d_robot(robot,robot.q);
    robot.q = inverse_kinematics_2d_robot(robot);
    
    robot.J = jacobian_2d_robot(robot,robot.q);
    % robot.J = jacobian_2d_robot_simple(robot,robot.q);
    
    
    %%%   sensor
    acc_temp=robot.g+(robot.f(1:3)+robot.f(4:6))./robot.body_mass;
    w_temp = robot.w;
    imu_temp = [w_temp;acc_temp];
    enc_temp = robot.q;
%     tau_temp = [robot.J((1:3),(1:2))'*robot.R'*robot.f(1:3);robot.J((1:3),(3:4))'*robot.R'*robot.f(4:6)];
    tau_temp = [robot.J((1:3),(1:2))'*robot.R'*temp_f(1:3);robot.J((1:3),(3:4))'*robot.R'*temp_f(4:6)];
    
    IMU(:,i) = imu_simulate(imu_temp,robot);
    ENC(:,i) = encoder_simulate(enc_temp,robot);
    TORQUE(:,i) = torquesensor_simulate(tau_temp,robot);
    TRUE_F(:,i) = [robot.R' zeros(3,3);zeros(3,3) robot.R']*robot.f;
    %     J_ENC = jacobian_2d_robot(robot,ENC(:,i+1));
    %     ROT(:,:,i) = robot.R;
    
    %%% FORCE = J*inv(J'*J) * TORQUE
    
    
    
    
    
    
    if i==1
        initial_R_estimate=eye(3);
        %         initial_R_estimate=robot.R;
        initial_p_estimate=robot.x;
        initial_v_estimate=robot.v;
        
    end
    
    
    if mod(i,1)==0
        draw_robot(fig,robot);
        
    end
    % pause(0.01)
end


%% fmincon
%
% options = optimoptions(@fmincon,'MaxFunctionEvaluations',100000,'Algorithm','interior-point','FiniteDifferenceType','central','HessianApproximation','lbfgs','SubproblemAlgorithm','cg');
options = optimoptions(@fmincon,'MaxFunctionEvaluations',100000);
fun_wo = @(x_wo) optimization_cost_wo(x_wo,robot,TORQUE,ENC,IMU,initial_R_estimate,initial_p_estimate,initial_v_estimate);
fun_dyn = @(x_dyn) optimization_cost_dyn(x_dyn,robot,TORQUE,ENC,IMU,initial_R_estimate,initial_p_estimate,initial_v_estimate,0,force_option);
% fun_dyn_gravity = @(x_dyn_gravity) optimization_cost_dyn(x_dyn_gravity,robot,TORQUE,ENC,IMU,initial_R_estimate,initial_p_estimate,initial_v_estimate,1,force_option);


if force_option==0
    for i=1:for_num
        x0_wo(:,i)=zeros(7,1);
        x0_dyn(:,i)=zeros(7,1);
    end
else
    for i=1:for_num
        x0_wo(:,i)=zeros(7,1);
        x0_dyn(:,i)=zeros(11,1);
    end
end

% dat = load('sol.mat');
% x0 = dat.x_wo;

tic
[x_wo,cost1] = fmincon(fun_wo,x0_wo,[],[],[],[],[],[],[],options);% conventional
time1 = toc
cost1

tic
[x_dyn,cost2] = fmincon(fun_dyn,x0_dyn,[],[],[],[],[],[],[],options);% algorithm1
time2 =toc
cost2


% tic
% [x_dyn_gravity,cost3] = fmincon(fun_dyn_gravity,x0_dyn,[],[],[],[],[],[],[],options);% algorithm1 + algorithm2
% time3 =toc
% cost3

%% lsqnonlin
% tic
% options.Algorithm = 'levenberg-marquardt';
% [x_dyn,cost1] = lsqnonlin(fun_dyn,x0,[],[],options);% x0,A,b ect. properly configured
% time1 =toc;
% cost1
% % tic
% % [x_wo,cost2] = fmincon(fun_wo,x0,[],[],[],[],[],[],[],options);% x0,A,b ect. properly configured
% % time2 = toc;
% % cost2
%








%% snopt

%
%
% %Get condensed data for the Hexagon problem.
%
%
%
% for i=1:for_num-1
%
%     %         x0(:,i)=initial_f;
%     %         AA=blkdiag(AA,A);
%     x0(:,i)=zeros(7,1);
% end
%
%
% % dat = load('sol.mat');
% % x0 = dat.x_wo;
%
%
% x_dyn=reshape(x0,[size(x0,1)*size(x0,2),1]);
% x_wo=reshape(x0,[size(x0,1)*size(x0,2),1]);
%
%
% neF    = 1;
% n      =  7*(for_num-1);
% Obj    =  1; % The default objective row
%
% % Ranges for F.
% % The Objective row (row 1 by default) is free.
%
% Fmul = zeros(neF,1);  Fstate = zeros(neF,1);
% Flow = zeros(neF,1);    Fupp = 10000000*ones (neF,1);
%
%
% % Ranges for x.
%
% xmul = zeros(n,1);      xstate = zeros(n,1);
% xlow = -Inf*ones(n,1);  xupp =  Inf*ones(n,1);
% snseti ('Major Iteration limit', 10000);
%
%
%
%
% tic
% [x_dyn,F,INFO] = snopt(x_dyn,xlow,xupp,xmul,xstate,...
%     Flow,Fupp,Fmul,Fstate,@(x) snopt_optimization_cost_dyn(x,robot,TORQUE,ENC,IMU,ROT,R_initial));
% time1 =toc
% F
% tic
% [x_wo,F,INFO] = snopt(x_wo,xlow,xupp,xmul,xstate,...
%     Flow,Fupp,Fmul,Fstate,@(x) snopt_optimization_cost_wo(x,robot,TORQUE,ENC,IMU,ROT,R_initial));
% time2 =toc
% F
% x_dyn = reshape(x_dyn,[7,for_num-1]);
% x_wo = reshape(x_wo,[7,for_num-1]);
%% Result

output_num = size(y_out,1);
for i=1:output_num
    R_true(:,:,i) = EUL_ZYX_to_R_bw(y_out(i,1:3));
end



R_estimate_wo(:,:,1) = initial_R_estimate;
v_estimate_wo(:,1) = initial_v_estimate;
p_estimate_wo(:,1)= initial_p_estimate;
for i=1:for_num
    
    R_estimate_wo(:,:,i+1) = initial_R_estimate*expm_vec(x_wo(1:3,i));
    v_estimate_wo(:,i+1) = [x_wo(4,i);0;x_wo(5,i)];
    p_estimate_wo(:,i+1) = [x_wo(6,i);0;x_wo(7,i)];
end

R_estimate_dyn(:,:,1) = initial_R_estimate;
v_estimate_dyn(:,1) = initial_v_estimate;
p_estimate_dyn(:,1)= initial_p_estimate;
for i=1:for_num
    R_estimate_dyn(:,:,i+1) = initial_R_estimate*expm_vec(x_dyn(1:3,i));
    v_estimate_dyn(:,i+1) = [x_dyn(4,i);0;x_dyn(5,i)];
    p_estimate_dyn(:,i+1) = [x_dyn(6,i);0;x_dyn(7,i)];
end

% R_estimate_dyn_gravity(:,:,1) = initial_R_estimate;
% v_estimate_dyn_gravity(:,1) = initial_v_estimate;
% p_estimate_dyn_gravity(:,1)= initial_p_estimate;
% for i=1:for_num
%     R_estimate_dyn_gravity(:,:,i+1) = initial_R_estimate*expm_vec(x_dyn_gravity(1:3,i));
%     v_estimate_dyn_gravity(:,i+1) = [x_dyn_gravity(4,i);0;x_dyn_gravity(5,i)];
%     p_estimate_dyn_gravity(:,i+1) = [x_dyn_gravity(6,i);0;x_dyn_gravity(7,i)];
% end






for i=1:(for_num+1)
    temp=logm_vec(R_true(:,:,10*(i)+1));
    real_theta(i) = temp(2);
    
    temp=logm_vec(R_estimate_wo(:,:,i));
    estimated_theta_wo(i) = temp(2);
    
    temp=logm_vec(R_estimate_dyn(:,:,i));
    estimated_theta_dyn(i) = temp(2);
    
%     temp=logm_vec(R_estimate_dyn_gravity(:,:,i));
%     estimated_theta_dyn_gravity(i) = temp(2);
    
    temp=logm_vec(R_estimate_wo(:,:,i)'*R_true(:,:,10*(i)+1));
    error_estimate_wo(i) = temp(2);
    
    temp=logm_vec(R_estimate_dyn(:,:,i)'*R_true(:,:,10*(i)+1));
    error_estimate_dyn(i) = temp(2);
    
%     temp=logm_vec(R_estimate_dyn_gravity(:,:,i)'*R_true(:,:,10*(i)+1));
%     error_estimate_dyn_gravity(i) = temp(2);
    
end
% for i=1:(for_num+1)
%
%     real_theta(i) = norm(logm_vec(R_true(:,:,10*(i-1)+1)));
%     estimated_theta_wo(i) = norm(logm_vec(R_estimate_wo(:,:,i)));
%     estimated_theta_dyn(i) = norm(logm_vec(R_estimate_dyn(:,:,i)));
%     estimated_theta_dyn_gravity(i) = norm(logm_vec(R_estimate_dyn_gravity(:,:,i)));
%     error_estimate_wo(i) = norm(logm_vec(R_estimate_wo(:,:,i)'*R_true(:,:,10*(i-1)+1)));
%     error_estimate_dyn(i) = norm(logm_vec(R_estimate_dyn(:,:,i)'*R_true(:,:,10*(i-1)+1)));
%     error_estimate_dyn_gravity(i) = norm(logm_vec(R_estimate_dyn_gravity(:,:,i)'*R_true(:,:,10*(i-1)+1)));
%
% end




%% plot

t=dt:dt:dt*(for_num+1);
t_rot=dt:dt:dt*(for_num+1);


h1 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_rot,real_theta,"k--","LineWidth",1.5);
hold on

plot(t_rot,estimated_theta_dyn,"b-","LineWidth",1.5);
plot(t_rot,estimated_theta_wo,"r-","LineWidth",1.5);
lgd = legend('True (rad)','Estimation with Dynamics (rad)','Estimation without Dynamics (rad)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Orientation in angle axis (rad)','FontSize',20,'FontName','Times New Roman')

set(h1,'Units','Inches');
pos = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h1,['Orientation Result ' num2str(T)],'-dpdf','-r0')


h2 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_rot,error_estimate_dyn,"b-","LineWidth",1.5);
hold on
plot(t_rot,error_estimate_wo,"r-","LineWidth",1.5);
lgd = legend('Error with Dynamics (rad)','Error without Dynamics (rad)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Error in angle axis(rad)','FontSize',20,'FontName','Times New Roman')

set(h2,'Units','Inches');
pos = get(h2,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h2,['Orientation Error Result ' num2str(T)],'-dpdf','-r0')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

h3 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_out,y_out(:,10),"k--","LineWidth",1.5);
hold on

plot(t,v_estimate_dyn(1,:),"b-","LineWidth",1.5);
plot(t,v_estimate_wo(1,:),"r-","LineWidth",1.5);
lgd = legend('True (m/s)','Estimation with Dynamics (m/s)','Estimation without Dynamics (m/s)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Velocity_X (m/s)','FontSize',20,'FontName','Times New Roman')

set(h3,'Units','Inches');
pos = get(h3,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h3,['Velocity_X Result ' num2str(T)],'-dpdf','-r0')


h4 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_out,y_out(:,7),"k--","LineWidth",1.5);
hold on

plot(t,p_estimate_dyn(1,:),"b-","LineWidth",1.5);
plot(t,p_estimate_wo(1,:),"r-","LineWidth",1.5);
lgd = legend('True (m)','Estimation with Dynamics (m)','Estimation without Dynamics (m)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Position_X (m)','FontSize',20,'FontName','Times New Roman')

set(h4,'Units','Inches');
pos = get(h4,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h4,['Position_X Result ' num2str(T)],'-dpdf','-r0')

h5 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_out,y_out(:,12),"k--","LineWidth",1.5);
hold on

plot(t,v_estimate_dyn(3,:),"b-","LineWidth",1.5);
plot(t,v_estimate_wo(3,:),"r-","LineWidth",1.5);
lgd = legend('True (m/s)','Estimation with Dynamics (m/s)','Estimation without Dynamics (m/s)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Velocity_Z (m/s)','FontSize',20,'FontName','Times New Roman')

set(h5,'Units','Inches');
pos = get(h5,'Position');
set(h5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h5,['Velocity_Z Result ' num2str(T)],'-dpdf','-r0')


h6 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
plot(t_out,y_out(:,9),"k--","LineWidth",1.5);
hold on

plot(t,p_estimate_dyn(3,:),"b-","LineWidth",1.5);
plot(t,p_estimate_wo(3,:),"r-","LineWidth",1.5);
lgd = legend('True (m)','Estimation with Dynamics (m)','Estimation without Dynamics (m)','location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Position_Z (m)','FontSize',20,'FontName','Times New Roman')

set(h6,'Units','Inches');
pos = get(h6,'Position');
set(h6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h6,['Position_Z Result ' num2str(T)],'-dpdf','-r0')


if force_option ==1

h7 = figure('Position',[1000,500,1200*0.6,600*0.6]);
R2D = 180/pi;
p1=plot(t,TRUE_F(1,:),"k--","LineWidth",1.5);
hold on
plot(t,TRUE_F(3,:),"k--","LineWidth",1.5);
plot(t,TRUE_F(4,:),"k--","LineWidth",1.5);
plot(t,TRUE_F(6,:),"k--","LineWidth",1.5);

p2=plot(t(1:(end-1)),x_dyn(8,:),"b-","LineWidth",1.5);
plot(t(1:(end-1)),x_dyn(9:11,:),"b-","LineWidth",1.5);
lgd = legend([p1,p2],{'True (N)','Estimation with Dynamics and imu (N)'},'location','Northeast');
set(lgd,'FontSize',13,'FontName','Times New Roman');
xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
ylabel('Force (N)','FontSize',20,'FontName','Times New Roman')

set(h7,'Units','Inches');
pos = get(h7,'Position');
set(h7,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])

T = 12;
print(h7,['Force_estimation_result ' num2str(T)],'-dpdf','-r0')

end

%% algorithm difference

% t=0:dt:dt*(for_num);
% t_rot=0:dt:dt*(for_num);
%
%
% h1 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_rot,real_theta,"k--","LineWidth",1.5);
% hold on
%
% plot(t_rot,estimated_theta_dyn,"b-","LineWidth",1.5);
% plot(t_rot,estimated_theta_wo,"r-","LineWidth",1.5);
%
% plot(t_rot,estimated_theta_dyn_gravity,"g-","LineWidth",1.5);
%
% lgd = legend('True (rad)','Estimation with Dynamics (rad)','Estimation without Dynamics (rad)','Estimation with Gravity (rad)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Orientation in angle axis (rad)','FontSize',20,'FontName','Times New Roman')
%
% set(h1,'Units','Inches');
% pos = get(h1,'Position');
% set(h1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h1,['Orientation Result ' num2str(T)],'-dpdf','-r0')
%
%
% h2 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_rot,error_estimate_dyn,"b-","LineWidth",1.5);
% hold on
% plot(t_rot,error_estimate_wo,"r-","LineWidth",1.5);
% plot(t_rot,error_estimate_dyn_gravity,"g-","LineWidth",1.5);
% lgd = legend('Error with Dynamics (rad)','Error without Dynamics (rad)','Error with Gravity (rad)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Error in angle axis(rad)','FontSize',20,'FontName','Times New Roman')
%
% set(h2,'Units','Inches');
% pos = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h2,['Orientation Error Result ' num2str(T)],'-dpdf','-r0')
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% h3 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_out,y_out(:,10),"k--","LineWidth",1.5);
% hold on
%
% plot(t,v_estimate_dyn(1,:),"b-","LineWidth",1.5);
% plot(t,v_estimate_wo(1,:),"r-","LineWidth",1.5);
% plot(t,v_estimate_dyn_gravity(1,:),"g-","LineWidth",1.5);
% lgd = legend('True (m/s)','Estimation with Dynamics (m/s)','Estimation without Dynamics (m/s)','Estimation with Gravity (m/s)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Velocity_X (m/s)','FontSize',20,'FontName','Times New Roman')
%
% set(h3,'Units','Inches');
% pos = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h3,['Velocity_X Result ' num2str(T)],'-dpdf','-r0')
%
%
% h4 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_out,y_out(:,7),"k--","LineWidth",1.5);
% hold on
%
% plot(t,p_estimate_dyn(1,:),"b-","LineWidth",1.5);
% plot(t,p_estimate_wo(1,:),"r-","LineWidth",1.5);
% plot(t,p_estimate_dyn_gravity(1,:),"g-","LineWidth",1.5);
% lgd = legend('True (m)','Estimation with Dynamics (m)','Estimation without Dynamics (m)','Estimation with Gravity (m)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Position_X (m)','FontSize',20,'FontName','Times New Roman')
%
% set(h4,'Units','Inches');
% pos = get(h4,'Position');
% set(h4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h4,['Position_X Result ' num2str(T)],'-dpdf','-r0')
%
% h5 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_out,y_out(:,12),"k--","LineWidth",1.5);
% hold on
%
% plot(t,v_estimate_dyn(3,:),"b-","LineWidth",1.5);
% plot(t,v_estimate_wo(3,:),"r-","LineWidth",1.5);
% plot(t,v_estimate_dyn_gravity(3,:),"g-","LineWidth",1.5);
% lgd = legend('True (m/s)','Estimation with Dynamics (m/s)','Estimation without Dynamics (m/s)','Estimation with Gravity (m/s)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Velocity_Z (m/s)','FontSize',20,'FontName','Times New Roman')
%
% set(h5,'Units','Inches');
% pos = get(h5,'Position');
% set(h5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h5,['Velocity_Z Result ' num2str(T)],'-dpdf','-r0')
%
%
% h6 = figure('Position',[1000,500,1200*0.6,600*0.6]);
% R2D = 180/pi;
% plot(t_out,y_out(:,9),"k--","LineWidth",1.5);
% hold on
%
% plot(t,p_estimate_dyn(3,:),"b-","LineWidth",1.5);
% plot(t,p_estimate_wo(3,:),"r-","LineWidth",1.5);
% plot(t,p_estimate_dyn_gravity(3,:),"g-","LineWidth",1.5);
% lgd = legend('True (m)','Estimation with Dynamics (m)','Estimation without Dynamics (m)','Estimation with Gravity (m)','location','Northeast');
% set(lgd,'FontSize',13,'FontName','Times New Roman');
% xlabel('Time (sec)','FontSize',20,'FontName','Times New Roman')
% ylabel('Position_Z (m)','FontSize',20,'FontName','Times New Roman')
%
% set(h6,'Units','Inches');
% pos = get(h6,'Position');
% set(h6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)+0.5])
%
% T = 12;
% print(h6,['Position_Z Result ' num2str(T)],'-dpdf','-r0')
%
