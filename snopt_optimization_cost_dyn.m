function f =snopt_optimization_cost_dyn(x_,robot,TORQUE,ENC,IMU,ROT,R_initial)
%OPTIMIZATION_COST 이 함수의 요약 설명 위치
%   자세한 설명 위치
size_num = size(x_,1);
x=reshape(x_,[7,size_num/7]);
for_num = size(TORQUE,2);

f=0;
for i=1:for_num-1
    torque = TORQUE(:,i);
    R=R_initial;
    v=[0;0];
    for j=1:i-1
    R = R*expm_vec(x(1:3,j));
    v= v + x(4:5,j);
    end
    v = [v(1);0;v(2)];
%     [residual_R,cov_R] = residual_R_dyn(R,x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_R,cov_R] = residual_R_wo(R,x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v,cov_v] = residual_v_dyn(R,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity
    [residual_p,cov_p] = residual_p_dyn(R,v,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position
    [residual_v2,cov_v2] = residual_v_wo(R,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity_wo
    [residual_p2,cov_p2] = residual_p_wo(R,v,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position_wo
    

%     f=f+residual_R'*inv(cov_R)*residual_R+residual_v'*inv(cov_v)*residual_v+residual_p'*inv(cov_p)*residual_p+residual_v2'*inv(cov_v2)*residual_v2+residual_p2'*inv(cov_p2)*residual_p2;
    
%     f=f+residual_R'*inv(cov_R)*residual_R+residual_v2'*inv(cov_v2)*residual_v2+residual_p2'*inv(cov_p2)*residual_p2;
f=f+residual_R'*inv(cov_R)*residual_R+residual_v'*inv(cov_v)*residual_v+residual_v2'*inv(cov_v2)*residual_v2+residual_p2'*inv(cov_p2)*residual_p2;
    




    

end

end

