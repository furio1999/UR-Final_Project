function A=fact_matrix(B, q, dq)
for i=1:length(q)
   bi=B(:,i);
   Ci=(1/2)*(jacobian(bi,q)+jacobian(bi,q)'-diff(B,q(i)));
   S(i,:)=dq'*Ci;
end
A=simplify(S);
end