% a=[1;2;3];
% b=[3;2;1];
% 
% A=hat_so3(a)+hat_so3(b);
% B=hat_so3(a+b);
% 
% 
% theta = [pi/4;pi/2;pi/4];
% 
% R=eye(3);
% 
% R1 = EUL_ZYX_to_R_wb(theta)
% 
% g = [0;0;-9.81];
% a = [2;2;2];
% g1 = R1'*g;
% a1 = R1'*[2;2;2];
% 
% g1 + a1
% 
% expm_vec((g + a) - (g1 + a1))

% J = jacobian_2d_robot_simple(robot,ENC(:,i));
% inv(J')*TORQUE(:,i)

% J_ENC((1:3),(1:2))*inv(J_ENC((1:3),(1:2))'*J_ENC((1:3),(1:2)))*TORQUE((1:2),i)
% inv([J_ENC((1),(1:2));J_ENC((3),(1:2))]')*TORQUE(1:2,i)

% out = cov_R(ROT(:,:,50),IMU(:,50),TORQUE(:,50),ENC(:,50),robot)


% [out,cov] = residual_R(ROT(:,:,49),ROT(:,:,50),IMU(:,49),ENC(:,49),TORQUE(:,49),robot)

J = jacobian_2d_robot_simple(robot,ENC(:,i));
J2 = [J(1,:);zeros(1,4);J(2,:);J(3,:);zeros(1,4);J(4,:)];
JJ = inv(J');
JJJ = [JJ(1,:);zeros(1,4);JJ(2,:);JJ(3,:);zeros(1,4);JJ(4,:)];
EYE = [eye(3) eye(3)];
f = EYE*JJJ*J2'*TRUE_F(:,i)*0.001/20
f_1 = 0.001/20*EYE*TRUE_F(:,i)
