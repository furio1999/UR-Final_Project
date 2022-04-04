function T=ParamMatrix(V, x)
for j=1:length(V)
    for i=1:length(x)
        T(j,i)=diff(V(j), x(i)); %T=col{ dA/dxi } i=1...n %maybe A(j, :)
    end
end
end