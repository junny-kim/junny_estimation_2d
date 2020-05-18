function f =optimization_cost_wo(x,robot,TORQUE,ENC,IMU,R_initial,p_initial,v_initial)
%OPTIMIZATION_COST 이 함수의 요약 설명 위치
%   자세한 설명 위치

for_num = size(TORQUE,2)-1;

R_cost=0;
R_tau_cost=0;
v_cost=0;
p_cost=0;

for i=1:for_num
    

%     [residual_R,cov_R] = residual_R_dyn(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation


if i==1
    R = R_initial;
    p_init = [p_initial(1);p_initial(3)];
    v_init = [v_initial(1);v_initial(3)];
    [residual_R,cov_R] = residual_R_wo(R,zeros(3,1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v,cov_v] = residual_v_wo(R,v_init,x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity
    [residual_p,cov_p] = residual_p_wo(R,v_initial,p_init,x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position
    [residual_R_tau,cov_R_tau] = residual_R_torque(R,IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
else
    R = R_initial*expm_vec(x(1:3,i-1));
    v= x(4:5,i-1);
    v= [v(1);0;v(2)];
    [residual_R,cov_R] = residual_R_wo(R,x(1:3,i-1),x(1:3,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
    [residual_v,cov_v] = residual_v_wo(R,x(4:5,i-1),x(4:5,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% velocity
    [residual_p,cov_p] = residual_p_wo(R,v,x(6:7,i-1),x(6:7,i),IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% position
    [residual_R_tau,cov_R_tau] = residual_R_torque(R,IMU(:,i),ENC(:,i),TORQUE(:,i),robot); %% orientation
end
    
    
    
    
    
    
R_tau_cost = R_tau_cost+ residual_R_tau'*inv(cov_R_tau)*residual_R_tau;    
R_cost = R_cost+ residual_R'*inv(cov_R)*residual_R;
v_cost = v_cost + residual_v'*inv(cov_v)*residual_v;
p_cost = p_cost + residual_p'*inv(cov_p)*residual_p;




    

end


% R_cost
% v_cost
% p_cost

f=R_cost + v_cost + p_cost;
% f=R_cost + v_cost + p_cost + R_tau_cost;


end

