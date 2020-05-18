function out = right_jacobian_so3(vec)
%RIGHT_JACOBIAN_SO3 �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
nor = norm(vec);
if nor < 1e-6
    out = eye(3);
else
    out = eye(3) - (1-cos(nor))/(nor^2)*hat_so3(vec)+(nor-sin(nor))/(norm(vec.^3))*hat_so3(vec)*hat_so3(vec);
end

