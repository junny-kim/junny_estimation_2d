function imu = imu_simulate(imu_temp,robot)
%IMU_SIMULATE 이 함수의 요약 설명 위치
%   자세한 설명 위치
imu = zeros(6,1);
imu(1:3) = imu_temp(1:3) + mvnrnd(zeros(3,1),robot.gyro_cov)';

imu(4:6) = robot.R'*(imu_temp(4:6) + robot.g)+mvnrnd(zeros(3,1),robot.acc_cov)';

g=9.81*10;

if imu(4)> g
    imu(4) = g;
end
if imu(4)< (-g)
    imu(4) = -g;
end

if imu(5)> (g)
    imu(5) = g;
end
if imu(5)< (-g)
    imu(5) = -g;
end

if imu(6)> (g)
    imu(6) = g;
end
if imu(6)< (-g)
    imu(6) = -g;
end

end