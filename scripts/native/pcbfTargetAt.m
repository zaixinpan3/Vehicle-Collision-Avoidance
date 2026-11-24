function target=pcbfTargetAt(root,time)
    if root(4)==0,target=root;else,target=predictiveSafetyGeometry.predictTarget(root,time);end
end
