function [residual,cov] = residual_v_wo(R,vec_bef,vec,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

a = IMU(4:6);

dt = robot.dt;
v=vec-vec_bef;
v=[v(1);0;v(2)];
residual = (v + robot.g*dt) - R*a*dt;

c1 = dt*eye(3)*R;
cov = c1*robot.acc_cov*c1';
residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];


end