function f =optimization_cost(x,robot,TORQUE,ENC,IMU,ROT)
%OPTIMIZATION_COST 이 함수의 요약 설명 위치
%   자세한 설명 위치

for_num = size(TORQUE,2);

f=0;
for i=1:for_num
    torque = TORQUE(:,i);
    J = jacobian_2d_robot(robot,ENC(:,i));

    a=IMU((4:6),i);
    R=ROT(:,:,i);
    
    
    
    
    
    
%     c1 = x(1:3,i)-J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2)))*torque((1:2));
%     cov1 = (J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2))))*robot.torque_cov((1:2),(1:2))*(J((1:3),(1:2))*inv(J((1:3),(1:2))'*J((1:3),(1:2))))';
%         cov1(2,2) = cov1(1,1);
%     c2 = x(4:6,i)-J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4)))*torque((3:4));
%     cov2 = (J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4))))*robot.torque_cov((3:4),(3:4))*(J((1:3),(3:4))*inv(J((1:3),(3:4))'*J((1:3),(3:4))))';
%         cov2(2,2) = cov2(1,1);
%     c3 = x(1:3,i)+x(4:6,i)+robot.body_mass.*(-R*a+2*robot.g);
%     cov3 = R*(robot.body_mass*robot.acc_cov)*R';
%     
%     f=f+c1'*inv(cov1)*c1+c2'*inv(cov2)*c2+c3'*inv(cov3)*c3;
    
    
    
    
    
    tau_temp = [J((1:3),(1:2))'*x((1:3),i);J((1:3),(3:4))'*x((4:6),i)];
    c1=torque-tau_temp;
    f=f+(c1)'*inv(robot.torque_cov)*(c1);
    a=IMU((4:6),i);
    R=ROT(:,:,i);
    I=[eye(3) eye(3)];
    c2=a-R'*(I*x(:,i)./robot.body_mass+2.*robot.g);
    f=f+(c2)'*inv(robot.acc_cov)*(c2);
    




    

end
end

