function f =optimization_cost_dyn(x,robot,TORQUE,ENC,IMU,R_initial,p_initial,v_initial,mode,force_option)
%OPTIMIZATION_COST 이 함수의 요약 설명 위치
%   자세한 설명 위치

for_num = size(TORQUE,2)-1;

R_cost=0;
R_tau_cost=0;
v_cost=0;
p_cost=0;
v_cost_2=0;
p_cost_2=0;
f_cost = 0;
for i=1:for_num
    

    
    
%     [residual_R,cov_R] = residual_R_dyn(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation


if i==1
    R = R_initial;
    p_init = [p_initial(1);p_initial(3)];
    v_init = [v_initial(1);v_initial(3)];
    
    
    if force_option==0
    [residual_R,cov_R] = residual_R_dyn(R,zeros(3,1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v,cov_v] = residual_v_dyn(R,v_init,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity
    [residual_p,cov_p] = residual_p_dyn(R,v_initial,p_init,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position
    else
    [residual_R,cov_R] = residual_R_dyn_force(R,zeros(3,1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% orientation
    [residual_v,cov_v] = residual_v_dyn_force(R,v_init,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% velocity
    [residual_p,cov_p] = residual_p_dyn_force(R,v_initial,p_init,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% position        
    end
    
    
%     [residual_R,cov_R] = residual_R_wo(R,zeros(3,1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_R_tau,cov_R_tau] = residual_R_torque(R,IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v2,cov_v2] = residual_v_wo(R,v_init,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity_wo
    [residual_p2,cov_p2] = residual_p_wo(R,v_initial,p_init,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position_wo
    
else
    R = R_initial*expm_vec(x(1:3,i-1));
    v= x(4:5,i-1);
    v= [v(1);0;v(2)];
    
    if force_option==0
    [residual_R,cov_R] = residual_R_dyn(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v,cov_v] = residual_v_dyn(R,x(4:5,i-1),x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity
    [residual_p,cov_p] = residual_p_dyn(R,v,x(6:7,i-1),x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position
    else
    [residual_R,cov_R] = residual_R_dyn_force(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% orientation
    [residual_v,cov_v] = residual_v_dyn_force(R,x(4:5,i-1),x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% velocity
    [residual_p,cov_p] = residual_p_dyn_force(R,v,x(6:7,i-1),x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot,x(8:11,i)); %% position        
    end
    
    
%     [residual_R,cov_R] = residual_R_wo(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_R_tau,cov_R_tau] = residual_R_torque(R,IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v2,cov_v2] = residual_v_wo(R,x(4:5,i-1),x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity_wo
    [residual_p2,cov_p2] = residual_p_wo(R,v,x(6:7,i-1),x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position_wo
    
end


R_cost = R_cost+ residual_R'*inv(cov_R)*residual_R;
R_tau_cost = R_tau_cost+ residual_R_tau'*inv(cov_R_tau)*residual_R_tau;
v_cost = v_cost + residual_v'*inv(cov_v)*residual_v;
p_cost = p_cost + residual_p'*inv(cov_p)*residual_p;
v_cost_2 = v_cost_2 + residual_v2'*inv(cov_v2)*residual_v2;
p_cost_2 = p_cost_2 + residual_p2'*inv(cov_p2)*residual_p2;


if force_option ==1
    
    [residual_f_acc,cov_f_acc] = residual_F_acc(R,x(8:11,i),IMU(:,i),robot); %% force_acc
    [residual_f_torque,cov_f_torque] = residual_F_torque(x(8:11,i),ENC(:,i),TORQUE(:,i),robot); %% force_torque

    
    f_cost = f_cost + residual_f_acc'*inv(cov_f_acc)*residual_f_acc + residual_f_torque'*inv(cov_f_torque)*residual_f_torque;
end





    

end

if mode==1
    f=R_cost + R_tau_cost + v_cost + p_cost + v_cost_2 + p_cost_2 + f_cost;
else
    f=R_cost + v_cost + p_cost + v_cost_2 + p_cost_2 + f_cost;
end

end

