    
x=zeros(6,100);
    
    torque = TORQUE(:,i);
    J = jacobian_2d_robot(robot,ENC(:,i));

    a=IMU((4:6),i);
    R=ROT(:,:,i);
    
    f=0;
    
    
    
    
    c1 = x(1:3,i)-J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2)))*torque((1:2));
    cov1 = (J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2))))*robot.torque_cov((1:2),(1:2))*(J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2))))'
    cov1(2,2) = cov1(1,1);
    c2 = x(4:6,i)-J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4)))*torque((3:4));
    cov2 = (J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4))))*robot.torque_cov((3:4),(3:4))*(J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4))))'
    cov2(2,2) = cov2(1,1);
    c3 = x(1:3,i)+x(4:6,i)-R*a+2*robot.g;
    cov3 = R*robot.acc_cov*R';
    
    f=f+c1'*inv(cov1)*c1+c2'*inv(cov2)*c2+c3'*inv(cov3)*c3