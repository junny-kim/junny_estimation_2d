function jacobian = jacobian_2d_robot(robot_model,q)
%JACOBIAN_2D_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치
jacobian_front_position=[robot_model.thigh_length*cos(q(1))+robot_model.calf_length*cos(q(1)+q(2)) robot_model.calf_length*cos(q(1)+q(2));
                         0 0;
                         -robot_model.thigh_length*sin(q(1))-robot_model.calf_length*sin(q(1)+q(2)) -robot_model.calf_length*sin(q(1)+q(2))];
jacobian_front_rotation=[0 0;1 1;0 0];
jacobian_rear_position=[robot_model.thigh_length*cos(q(3))+robot_model.calf_length*cos(q(3)+q(4)) robot_model.calf_length*cos(q(3)+q(4));
                         0 0;
                         -robot_model.thigh_length*sin(q(3))-robot_model.calf_length*sin(q(3)+q(4)) -robot_model.calf_length*sin(q(3)+q(4))];
jacobian_rear_rotation=[0 0;1 1;0 0];


jacobian = [jacobian_front_position, jacobian_rear_position;
            jacobian_front_rotation, jacobian_rear_rotation];
end

