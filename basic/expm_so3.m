function out = expm_so3(so3)
%EXPM_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치
vec = vee_so3(so3);
nor = norm(vec);

if NearZero(vec)
    out = eye(3);
else
out = eye(3) + sin(nor)/nor*so3 + (1-cos(nor))/(nor^2)*so3*so3;
end
end

function judge = NearZero(near)
judge=norm(near)<1e-10;
end