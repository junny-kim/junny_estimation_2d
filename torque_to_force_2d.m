function f = torque_to_force_2d(ENC,TORQUE,robot)
%TORQUE_TO_FORCE_2D �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
J = jacobian_2d_robot_simple(robot,ENC);
JJ = inv(J');
f_temp = JJ*TORQUE;
f = [f_temp(1);0;f_temp(2);f_temp(3);0;f_temp(4)];
end

