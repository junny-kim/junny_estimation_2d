clear all
close all
clc

leg_num = 2;
for_num = 50;
dt=0.01;

robot = robot_model(leg_num,dt);



options = optimoptions('fmincon','MaxFunctionEvaluations',100000);

addpath("I:\Dropbox\study\matlab_jh\manifold_functions")
addpath("basic")
% robot_1 = robot_model(leg_num);

q_log=[];
u_log=[];
robot.q = [-3*pi/4;-pi/2;-3*pi/4;-pi/2];
% robot.qdot = [1.0;0;0;0];
robot.qddot = [0.0;0;0;0];
robot.u = [0;0;0;0];
initial_f=[randn(1);0;20*4.905;0;randn(1);20*4.905];
robot.f = initial_f;
robot.x = [0;0;0];

robot.x_foot=forward_kinematics_2d_robot(robot,[-3*pi/4;-pi/2;-3*pi/4;-pi/2]);
front_foot_position = robot.x + robot.R'*robot.x_foot(1:3);
rear_foot_position = robot.x + robot.R'*robot.x_foot(4:6);
robot.J = jacobian_2d_robot(robot,robot.q);
cost_MAP=0;
out=[];


% robot.J'*

fig = figure(1);
draw_robot(fig,robot)
% switch leg_num
%     case 1
%     case 2
% end
% 

% dynamics_2d_robot(robot)
y_out=[];
t_out=[];
for i=1:for_num
% randn(1)
robot.f = [10*sin(i/10);0;20*4.905+40*sin(i/10);10*sin(i/10);0;20*4.905+20*sin(i/10)];
tspan = linspace(0,dt,11);
state_init = [robot.theta;robot.w;robot.x;robot.v];
[t,y]=ode45(@(t,y) dynamics_2d_robot(t,y,robot),tspan,state_init);
t_out=[t_out; dt*i+t((1:end-1),:)];
y_out=[y_out; y((1:end-1),:)];


%%%   sensor
acc_temp=robot.g+(robot.f(1:3)+robot.f(4:6))./robot.body_mass;
w_temp = robot.w;
imu_temp = [w_temp;acc_temp];
enc_temp = robot.q;
tau_temp = [robot.J((1:3),(1:2))'*robot.f(1:3);robot.J((1:3),(3:4))'*robot.f(4:6)];

IMU(:,i) = imu_simulate(imu_temp,robot);
ENC(:,i) = encoder_simulate(enc_temp,robot);
TORQUE(:,i) = torquesensor_simulate(tau_temp,robot);
TRUE_F(:,i) = robot.f;
J_ENC = jacobian_2d_robot(robot,ENC(:,i));
ROT(:,:,i) = robot.R;

%%% FORCE = J*inv(J'*J) * TORQUE




CHECK(:,i)=IMU((4:6),i)-robot.R'*((TRUE_F(1:3,i)+TRUE_F(4:6,i))./robot.body_mass+2.*robot.g);
ESTIMATE_F((1:3),i)=J_ENC((1:3),(1:2))*inv(J_ENC((1:3),(1:2))'*J_ENC((1:3),(1:2)))*TORQUE((1:2),i);
ESTIMATE_F((4:6),i)=J_ENC((1:3),(3:4))*inv(J_ENC((1:3),(3:4))'*J_ENC((1:3),(3:4)))*TORQUE((3:4),i);



%%%      update
robot.theta=y(end,(1:3))';
robot.R=EUL_ZYX_to_R_wb(robot.theta);
robot.w=y(end,(4:6))';
robot.x=y(end,(7:9))';
robot.v=y(end,(10:12))';
robot.q = inverse_kinematics_2d_robot(robot);
robot.x_foot(1:3) = robot.R'*(front_foot_position-robot.x);
robot.x_foot(4:6) = robot.R'*(rear_foot_position-robot.x);
robot.J = jacobian_2d_robot(robot,robot.q);
% robot.J = jacobian_2d_robot_simple(robot,robot.q);







if mod(i,10)==0
draw_robot(fig,robot);

end
% pause(0.01)
end





    fun = @(x) optimization_cost(x,robot,TORQUE,ENC,IMU,ROT);
    
    x0=zeros(6,for_num);
    A=zeros(6,6);
    A(2,2)=1;
    A(5,5)=1;
    AA=[];
    for i=1:for_num
        x0(:,i)=initial_f;
        AA=blkdiag(AA,A);
    end
    
    
    
    b=zeros(6*for_num,1);
   [x,val] = fmincon(fun,x0,[],[],AA,b,[],[],[],options);% x0,A,b ect. properly configured

t=dt:dt:dt*for_num;
figure(2)
clf
plot(t,TRUE_F(1,:));
hold on
plot(t,ESTIMATE_F(1,:))
plot(t,x(1,:))


figure(3)
clf
plot(t,TRUE_F(3,:));
hold on
plot(t,ESTIMATE_F(3,:))
plot(t,x(3,:))

figure(4)
clf
plot(t,TRUE_F(4,:));
hold on
plot(t,ESTIMATE_F(4,:))
plot(t,x(4,:))


figure(5)
clf
plot(t,TRUE_F(6,:));
hold on
plot(t,ESTIMATE_F(6,:))
plot(t,x(6,:))

figure(6)
clf
plot(t_out,y_out(:,(9)))

rms(TRUE_F'-ESTIMATE_F')
rms(TRUE_F'-x')
