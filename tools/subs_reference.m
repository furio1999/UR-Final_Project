function R=subs_reference(A, old, new)
R=A;
for i=1:length(new)
   R=subs(R, old(i), new(i));
end
R=simplify(R);
if isnumeric(R)
 R=double(R);
end
end