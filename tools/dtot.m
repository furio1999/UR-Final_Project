function res=dtot(A, x, dx)
res=zeros(size(A,1), size(A,2)); %column vector, dimension=number of rows of the matrix/vector A
for i=1:length(x)
    res;
    var=diff(A, x(i))*dx(i);
    res=res+var; %dM/dq=dM/dq1 + dM/dq2 + ... + dM/dqn
end
end  %total differentiation