function [idx] = findConnection(path,q)

conn = path.connections;
idx = 0;

for i=1:length(conn)
    
    parent = conn(i).getParent.q;
    child = conn(i).getChild.q;
    
    dist = norm(parent-child);
    dist1 = norm(parent-q);
    dist2 = norm(q-child);
    
    if(abs(dist-dist1-dist2)<1e-06)
        idx = i;
    end
end

end

