function [residual,cov] = residual_p_wo(R,v,vec_bef,vec,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

a = IMU(4:6);

dt = robot.dt;
p=vec-vec_bef;
p =[p(1);0;p(2)];
residual = (p + 0.5*robot.g*dt^2 -v*dt) - 0.5*R*a*dt^2;

c1 = 0.5*dt^2*eye(3)*R;
cov = c1*robot.acc_cov*c1';
residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];

end

