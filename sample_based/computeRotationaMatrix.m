function [M] = computeRotationaMatrix(v)
if norm(v)<1e-10
    error('v should not be null')
end
v=v/norm(v);
ndof=length(v);
M=eye(ndof);
for ic1=1:ndof
    if abs(dot(v,M(:,ic1)))>1-1e-10
        % v is a column of eye matrix, put this column as first
        M=M(:,[ic1 1:(ic1-1) (ic1+1):end]);
        return
    end
end

% if v is not a column of eye matrix
M(:,1)=v;
% run Gram-Schmidt orthogonalization
for ic1=2:ndof
    for ic2=1:(ic1-1)
        M(:,ic1)=M(:,ic1)-dot(M(:,ic1),M(:,ic2))*M(:,ic2);
        M(:,ic1)=M(:,ic1)/norm(M(:,ic1));
    end
end

end

