function jacobian = jacobian_2d_robot_simple(robot_model,q)
%JACOBIAN_2D_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치
jacobian_front_position=[robot_model.thigh_length*cos(q(1))+robot_model.calf_length*cos(q(1)+q(2)) robot_model.calf_length*cos(q(1)+q(2));
                        -robot_model.thigh_length*sin(q(1))-robot_model.calf_length*sin(q(1)+q(2)) -robot_model.calf_length*sin(q(1)+q(2))];

jacobian_rear_position=[robot_model.thigh_length*cos(q(3))+robot_model.calf_length*cos(q(3)+q(4)) robot_model.calf_length*cos(q(3)+q(4));
                         -robot_model.thigh_length*sin(q(3))-robot_model.calf_length*sin(q(3)+q(4)) -robot_model.calf_length*sin(q(3)+q(4))];



jacobian = [jacobian_front_position, zeros(2,2);
            zeros(2,2), jacobian_rear_position];
end

