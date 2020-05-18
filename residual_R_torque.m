function [residual,cov] = residual_R_torque(R,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

a = IMU(4:6);
dt = robot.dt;
m=robot.body_mass;
EYE = [eye(3) eye(3)];
J = jacobian_2d_robot_simple(robot,ENC);
% JJ = inv(J');
JJ = (J')\eye(4);
JJJ = [JJ(1,:);zeros(1,4);JJ(2,:);JJ(3,:);zeros(1,4);JJ(4,:)];
f = JJJ*TORQUE;
% f=TRUE_F;

residual = a - (1/m).*EYE*f -2*R'*robot.g;

c = (1/m).*EYE*JJJ;

cov = robot.acc_cov - c*robot.torque_cov*c';

end