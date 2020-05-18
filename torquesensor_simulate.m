function torque = torquesensor_simulate(tau_temp,robot)
%TORQUESENSOR_SIMULATE 이 함수의 요약 설명 위치
%   자세한 설명 위치


torque = tau_temp + mvnrnd(zeros(4,1),robot.torque_cov)';
end

