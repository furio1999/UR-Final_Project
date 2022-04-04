function A=Inertia(T, q, dq)
for i=1:length(dq)
    B(i, i)=diff(T,dq(i),2);
    for j=i:length(dq) 
        if (i ~= j)
            temp_ij=diff(T, dq(i));
            B(i,j)=simplify(diff(temp_ij, dq(j)));
            B(j,i)=B(i,j);         
        end
    end
end

A=simplify(B);
end