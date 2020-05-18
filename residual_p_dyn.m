function [residual,cov] = residual_p_dyn(R,v,vec_bef,vec,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치
dt = robot.dt;
m=robot.body_mass;
EYE = [eye(3) eye(3)];
p=vec-vec_bef;
p = [p(1);0;p(2)];
J = jacobian_2d_robot_simple(robot,ENC);
JJ = inv(J');
JJJ = [JJ(1,:);zeros(1,4);JJ(2,:);JJ(3,:);zeros(1,4);JJ(4,:)];
f = JJJ*TORQUE;

residual = R'*(p - v*dt -0.5*robot.g*dt^2)  - ((dt^2)/(2*m))*EYE*f;
% residual = R'*(p - v*dt);

c1 = (dt^2/(2*m))*EYE*JJJ;
cov = c1*robot.torque_cov*c1';
residual = [residual(1);residual(3)];
cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];

end
