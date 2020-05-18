function  state_dot= dynamics_2d_robot(t,y,robot)
%DYNAMICS_2D_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치

%[theta;w_b;x;v]


state_dot = [robot.w;robot.body_inertia\((hat_so3(robot.x_foot(1:3))*robot.R'*robot.f(1:3)+hat_so3(robot.x_foot(4:6))*robot.R'*robot.f(4:6))-hat_so3(robot.w)*(robot.body_inertia*robot.w));robot.v;robot.g+(robot.f(1:3)+robot.f(4:6))./robot.body_mass];
% state_dot = [y(4:6)';robot.body_inertia\(hat_so3(robot.x_foot(1:3))*robot.f(1:3)+hat_so3(robot.x_foot(4:6))*robot.f(4:6));y(10:12)';robot.g+(robot.f(1:3)+robot.f(4:6))./robot.body_mass]';


end