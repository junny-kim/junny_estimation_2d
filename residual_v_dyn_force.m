function [residual,cov] = residual_v_dyn_force(R,vec_bef,vec,IMU,ENC,TORQUE,robot,force)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치
dt = robot.dt;
m=robot.body_mass;
EYE = [eye(3) eye(3)];
v=vec-vec_bef;
v = [v(1);0;v(2)];
f = [force(1);0;force(2);force(3);0;force(4)];

residual = R'*(v -robot.g*dt) - (dt/m).*EYE*f;

c1 = (dt/m).*EYE;
cov = c1*robot.force_cov*c1';

residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];
end
