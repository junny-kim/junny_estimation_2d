function [residual,cov] = residual_v_dyn(R,vec_bef,vec,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치
dt = robot.dt;
m=robot.body_mass;
EYE = [eye(3) eye(3)];
v=vec-vec_bef;
v = [v(1);0;v(2)];
J = jacobian_2d_robot_simple(robot,ENC);
% JJ = inv(J');
JJ = (J')\eye(4);
JJJ = [JJ(1,:);zeros(1,4);JJ(2,:);JJ(3,:);zeros(1,4);JJ(4,:)];
f = JJJ*TORQUE;
% f=TRUE_F;

residual = R'*(v -robot.g*dt) - (dt/m).*EYE*f;
% residual = R'*(v);
% residual = (vec -robot.g*dt) - (dt/m).*EYE*blkdiag(R,R)*f;
% residual = (vec -robot.g*dt) - (dt/m).*EYE*f;

c1 = (dt/m).*EYE*JJJ;
% c1 = (dt/m).*EYE;
cov = c1*robot.torque_cov*c1';
% cov = c1*blkdiag(robot.gyro_cov,robot.acc_cov)*c1';
residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];
end
%    check = [50.0000;0;98.1000;50.0000;0;245.2500];