function encoder = encoder_simulate(enc_temp,robot)
%ENCODER_SIMULATE �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ


encoder = enc_temp + mvnrnd(zeros(4,1),robot.encoder_cov)';
end

