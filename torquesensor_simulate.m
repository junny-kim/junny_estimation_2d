function torque = torquesensor_simulate(tau_temp,robot)
%TORQUESENSOR_SIMULATE �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ


torque = tau_temp + mvnrnd(zeros(4,1),robot.torque_cov)';
end

