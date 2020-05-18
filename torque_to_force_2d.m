function f = torque_to_force_2d(ENC,TORQUE,robot)
%TORQUE_TO_FORCE_2D 이 함수의 요약 설명 위치
%   자세한 설명 위치
J = jacobian_2d_robot_simple(robot,ENC);
JJ = inv(J');
f_temp = JJ*TORQUE;
f = [f_temp(1);0;f_temp(2);f_temp(3);0;f_temp(4)];
end

