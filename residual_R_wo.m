function [residual,cov] = residual_R_wo(R,vec_bef,vec,IMU,ENC,TORQUE,robot)
%RESIDUAL_R 이 함수의 요약 설명 위치
%   자세한 설명 위치

w = IMU(1:3);
dt = robot.dt;

w_dt = dt*w;
residual = logm_vec(expm_vec(w_dt)'*(expm_vec(vec_bef)'*expm_vec(vec)));
% residual = residual * 10;
c2 = right_jacobian_so3(w_dt)*dt;

cov = c2*robot.gyro_cov*c2';

end

