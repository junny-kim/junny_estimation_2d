FVAL_SNOPT= [];
X_sdp = sdpvar(n,1);
cost_sdp = 0.5*X_sdp'*P*X_sdp+q'*X_sdp;
const_sdp = [0<=X_sdp<=bound];
opts = sdpsettings('solver','snopt','verbose',0,'savesolveroutput',1,'debug',1);
opts.usex0=1;
opts.showprogress = 1;
assign(X_sdp,zeros(n,1));
for ii=0:200
    disp(ii)
    string=['snseti (''Major Iteration limit'', ', num2str(ii) ,');'];
    eval(string);
    rehash;
    sol = optimize(const_sdp,cost_sdp,opts);
    FVAL_SNOPT(ii+1)=value(cost_sdp);
    if sol.problem==0
        break;
    end
end