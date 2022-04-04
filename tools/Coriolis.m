function vector=Coriolis(B, q, dq)
for i=1:length(q)
   bi=B(:,i);
   Ci=(1/2)*(jacobian(bi,q)+jacobian(bi,q)'-diff(B,q(i)));
   c(i)=dq'*Ci*dq;
end
vector=simplify(c');
end