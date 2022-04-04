function v = vc_diff(pc, q, dq)
v=zeros(length(pc), 1);
for i=1:length(q)
    v=v+diff(pc, q(i))*dq(i);
end
end