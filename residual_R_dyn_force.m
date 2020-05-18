function [residual,cov] = residual_R_dyn_force(R,vec_bef,vec,IMU,ENC,TORQUE,robot,force)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

w = IMU(1:3);
dt = robot.dt;
I = robot.body_inertia;
inv_I = inv(I);

J = jacobian_2d_robot_simple(robot,ENC);
JJ = inv(J');
JJJ = [JJ(1,:);zeros(1,4);JJ(2,:);JJ(3,:);zeros(1,4);JJ(4,:)];
f = [force(1);0;force(2);force(3);0;force(4)];

foot = forward_kinematics_2d_robot(robot,ENC);
r_hat = [hat_so3(foot(1:3)) hat_so3(foot(4:6))];

w_dt = dt*w+0.5*inv_I*(r_hat*f)*dt^2-0.5*inv_I*hat_so3(w)*I*w*dt^2;

residual = logm_vec(expm_vec(w_dt)'*expm_vec(vec_bef)'*expm_vec(vec));

c1 = 0.5*right_jacobian_so3(w_dt)*inv_I*r_hat*dt^2;
c2 = right_jacobian_so3(w_dt)*(dt*eye(3)-0.5*inv_I*(hat_so3(w)*I-hat_so3(I*w))*dt^2);
cov = c1*robot.force_cov*c1'+c2*robot.gyro_cov*c2';

end

