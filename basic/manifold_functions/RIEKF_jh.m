
function [R_out,P_out] = RIEKF_jh(R,P,Qg,Qa,dt,g,correction,a,w)


            gravity = g(3);
            
            % taking the a = acceleration and w angolar velocity from the u input from
            % the imu
            %a = u(3:end);
            %w = u(1:3);  % angular velocity
            %a = u(2:3);  % acceleration
        

            % prediction step---- update all the state using the actual data from the
      
            R_pred = R*expm(hat_so3(w.*dt));

            % Define the Adjoint matrix 

            Adj = R;

            % define Linearized dynamics matrix 

            A = zeros(3);
            % propagate the covariance matrix with the riccati equation 

            F  = eye(size(A)) + A*dt;
            cov_w = Qg;
            Q = F*Adj*cov_w*Adj'*F*dt;
            P_pred  = F*P*F' + Q;
            if correction < 0
                P_out=P_pred;
                R_out=R_pred;
               
            else
                
                % correction step 
                Y = a; 
                % kalman filter gain 
                H = [0 -gravity 0;...
                    gravity 0 0;...
                     0 0 0];
                N = Qa;
                S = H*P_pred*H' + N ;
                K = P_pred*H'/S ;
                R_out = expm(hat_so3(K*(R_pred*Y-g)))*R_pred;
%                 R_out = expm(hat_so3(K*(R_pred*g-g)))*R_pred;
                P_out = (eye(3) - K*H)*P_pred;

            end
            end