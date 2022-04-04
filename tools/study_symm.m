function value=study_symm(A, trigger)
value="yes";

if trigger=="skew"
for i=1:size(A,1)
    for j=1:size(A,2)
        if i~=j
           if simplify(A(i,j)) ~= - simplify(A(j,i))
               value="no";
               fprintf("not symmetric in: %i, %i\n", i, j)
               difference=A(i,j)+A(j,i);
               difference=simplify(difference)
               return 
           end
        end
    end
end
end %if

if trigger=="symm"
        
for i=1:size(A,1)
    for j=1:size(A,2)
        if i~=j
            if simplify(A(i,j)) ~= simplify(A(j,i))
               value="no";
               fprintf("not symmetric in: %i, %i", i, j);
               difference=A(i,j)-A(j,i);
               simplify(difference)
               return 
               end
        end
    end
 end

end
    
end