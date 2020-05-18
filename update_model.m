function update_model(robot,y)
robot.theta=y(end,(1:3))';
robot.R=[cos(robot.theta(2)) 0 sin(robot.theta(2)); 0 1 0; -sin(robot.theta(2)) 0 cos(robot.theta(2))];
robot.w=y(end,(4:6))';
robot.x=y(end,(7:9))';
robot.v=y(end,(10:12))';
end

