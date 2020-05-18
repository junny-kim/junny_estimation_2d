classdef robot_model
    %ROBOT_MODEL matlab class for robot model
    %   matlab class for robot model
    %   initialize with leg_num
    
    properties
        dt=0.01;
        body_mass=20;
        thigh_mass=4;
        calf_mass=4;
        thigh_length = 0.5;
        calf_length = 0.5;
        body_length = 1.0;
        body_height = 0.4;
        body_inertia;
        leg_num = 2;
        g=[0;0;-9.81];
        f;
        u;
        q;
        qdot;
        qddot;
        R = eye(3); %   R_0b
        theta = zeros(3,1);
        x = zeros(3,1);
        w = zeros(3,1);% w_b
        alpha = zeros(3,1);
        v = zeros(3,1);
        a = zeros(3,1);
        x_foot = zeros(6,1);
        J;
        J_f;
        J_f_R;
        J_r;
        J_r_R;
        gyro_cov = 0.01*eye(3);
        acc_cov = 0.01*eye(3);
        torque_cov = 0.1*eye(4);
        encoder_cov = 0.0*eye(4);
        force_cov = 0.1*eye(6);
    end
    
    methods
        function obj = robot_model(inputArg1,inputArg2)
            %ROBOT_MODEL 이 클래스의 인스턴스 생성
            %   자세한 설명 위치
            obj.leg_num = inputArg1;
            obj.dt = inputArg2;
            obj.q=zeros(obj.leg_num*2,1);
            obj.qdot=zeros(obj.leg_num*2,1);
            obj.qddot=zeros(obj.leg_num*2,1);
            obj.u=zeros(obj.leg_num*2,1);
            obj.f=zeros(obj.leg_num*3,1);
            obj.body_inertia=0.5*obj.body_mass*obj.body_length^2*eye(3);
            obj.force_cov(2,2)=0;
            obj.force_cov(4,4)=0;
        end
      
        
    end
end

