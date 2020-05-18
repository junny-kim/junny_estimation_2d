function [residual,cov] = residual_F_torque(force,ENC,TORQUE,robot)
%RESIDUAL_R �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
J = jacobian_2d_robot_simple(robot,ENC);

residual = J'* force - TORQUE;


% force
cov = robot.torque_cov;
% residual = [residual(1);residual(3)];
% cov = [cov(1,1) cov(1,3);cov(3,1) cov(3,3)];
% force
end