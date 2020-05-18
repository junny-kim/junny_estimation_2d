function  x = forward_kinematics_2d_robot(robot,ENC)
%FORWARD_KINEMATICS 이 함수의 요약 설명 위치
%   자세한 설명 위치
x_front = [0.4 + robot.thigh_length*sin(ENC(1)) + robot.calf_length*sin(ENC(1)+ENC(2));0;robot.thigh_length*cos(ENC(1))+robot.calf_length*cos(ENC(1)+ENC(2))];
x_rear = [-0.4 + robot.thigh_length*sin(ENC(3)) + robot.calf_length*sin(ENC(3)+ENC(4));0;robot.thigh_length*cos(ENC(3))+robot.calf_length*cos(ENC(3)+ENC(4))];
x=[x_front;x_rear];
end

