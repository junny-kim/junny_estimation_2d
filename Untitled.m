    fun = @(x) optimization_cost(x,robot,TORQUE,ENC,IMU,ROT);
    
    x0=zeros(6,for_num);
    A=zeros(6,6);
    A(2,2)=1;
    A(5,5)=1;
    AA=[];
    for i=1:for_num
        x0(:,i)=initial_f;
        AA=blkdiag(AA,A);
    end
    b=zeros(6*for_num,1);
    
    
    
   [x,val] = fmincon(fun,x0,[],[],AA,b,[],[],[],options);% x0,A,b ect. properly configured

t=dt:dt:dt*for_num;
figure(2)
clf
plot(t,TRUE_F(1,:));
hold on
plot(t,ESTIMATE_F(1,:))
plot(t,x(1,:))


figure(3)
clf
plot(t,TRUE_F(3,:));
hold on
plot(t,ESTIMATE_F(3,:))
plot(t,x(3,:))

figure(4)
clf
plot(t,TRUE_F(4,:));
hold on
plot(t,ESTIMATE_F(4,:))
plot(t,x(4,:))


figure(5)
clf
plot(t,TRUE_F(6,:));
hold on
plot(t,ESTIMATE_F(6,:))
plot(t,x(6,:))

figure(6)
clf
plot(t_out,y_out(:,(9)))

rms(TRUE_F'-ESTIMATE_F')
rms(TRUE_F'-x')