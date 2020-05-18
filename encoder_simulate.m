function encoder = encoder_simulate(enc_temp,robot)
%ENCODER_SIMULATE 이 함수의 요약 설명 위치
%   자세한 설명 위치


encoder = enc_temp + mvnrnd(zeros(4,1),robot.encoder_cov)';
end

