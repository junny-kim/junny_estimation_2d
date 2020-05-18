function  q = inverse_kinematics_2d_robot(robot)
%INVERSE_KINEMATICS 이 함수의 요약 설명 위치
%   자세한 설명 위치

front_foot = robot.x_foot(1:3)-[0.4;0;0];
rear_foot = robot.x_foot(4:6)-[-0.4;0;0];

% front_foot=robot.R*(robot.x_foot(1:3)-(robot.x+robot.R'*[0.4;0;0]));
% rear_foot=robot.R*(robot.x_foot(4:6)-(robot.x+robot.R'*[-0.4;0;0]));

x=front_foot(1);
z=front_foot(3);
q2 = acos((x^2+z^2-robot.thigh_length^2-robot.calf_length^2)/(2*robot.thigh_length*robot.calf_length));
if q2>0
    q2=-q2;
end




q1 = atan2(x,z)-atan2(robot.calf_length*sin(q2),(robot.thigh_length+robot.calf_length*cos(q2)));
x=rear_foot(1);
z=rear_foot(3);
q4 = acos((x^2+z^2-robot.thigh_length^2-robot.calf_length^2)/(2*robot.thigh_length*robot.calf_length));

if q4>0
    q4=-q4;
end
q3 = atan2(x,z)-atan2(robot.calf_length*sin(q2),(robot.thigh_length+robot.calf_length*cos(q2)));

% if q2<0
%     q2=-2*pi-q2;
% end
% if q4<0
%     q4=-2*pi-q4;
% end
% 
% 
% if q1<0
%     q1=-2*pi-q1;
% end
% 
% if q3<0
%     q3=-2*pi-q3;
% end
% 
% 
if q2>0
    q2=q2-2*pi;
end

if q4>0
    q4=q4-2*pi;
end

q=[q1;q2;q3;q4];
end

