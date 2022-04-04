syms x y z
%%aggiusta le R
Rx=[1 0 0;
   0 cos(x) -sin(x);
   0 sin(x) cos(x)];
Ry=[cos(y) 0 sin(y);
    0 1 0;
    -sin(y) 0 cos(y)];
Rz=[cos(z) -sin(z) 0;
    sin(z) cos(z) 0;
    0 0 1];

% syms t x
% fun= @(t) 3*t + 5;
% q=integral(fun, 0, inf)

%derivative part
% syms q1 q2 q3 real
% q=[q1 q2 q3]';
% p=[sin(q1)*5, cos(q2)*4,  sin(q1)*cos(q3)*sin(q2)]';
% J=jacobian(p, q)
% J33=diff(diff(J(3,3), q1), q3)
% J_dot=diff(diff(diff(J,q1), q2), q3)

syms q1 q2 dq1 dq2 ddq1 ddq2 real
syms l1 l2
q=[q1 q2]';
dq=[dq1 dq2]';
ddq=[ddq1 ddq2]';
p=[l1.*cos(q1)+l2*cos(q1+q2); l1.*sin(q2)+l2*sin(q1+q2)];
J=jacobian(p,q)
J_dot=diff(J, q1)*dq1+diff(J,q2)*dq2

% dp=[-q2.*sin(q1r).*dq1r+cos(q1r).*dq2; q2.*cos(q1r).*dq1r+sin(q1r).*dq2];
dp=simplify(J*dq)
% ddp=[-q2.*sin(q1r).*ddq1r+cos(q1r).*ddq2; 
%       q2.*cos(q1r).*ddq1r+sin(q1r).*ddq2];
ddp=J*ddq
% ddp=ddp+[-2*sin(q1r).*dq1r.*dq2-q2.*cos(q1r).*dq1r.^2; 
%           2*cos(q1r).*dq1r.*dq2-q2.*sin(q1r).*dq1r.^2];
ddp2=simplify(J_dot*dq)
ddp=simplify(ddp+ddp2);

syms a b c;
r=[a b c]';

function out = S(r)
  out=[0   -c   b ;
 c    0  -a ;
-b    a   0  ];
  
end