function [residual,cov] = residual_F_acc(R,force,IMU,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

a = IMU(4:6);
m=robot.body_mass;
EYE = [eye(3) eye(3)];
f = [force(1);0;force(2);force(3);0;force(4)];
residual=EYE*f./m+2.*R'*robot.g-a;


cov = robot.acc_cov;
residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];

end
