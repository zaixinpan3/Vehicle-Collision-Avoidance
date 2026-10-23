function result = nrmmTargetJacobian(action,varargin)
% nrmmTargetJacobian Full vector Jacobians of the retained NRMM extension.
% enclosure(domain) returns a finite analytic convex enclosure of [DqPhi,DsPhi]
% in the q-aligned frame, covering every radial/scalar/acceleration saturation.
% Orthogonal equivariance and P kron I2 make it sufficient for all headings.
% The vertices are polynomial/rational Bernstein coefficients, not samples.
% evaluate(q,s,domain) returns the Cartesian Jacobian on a smooth branch.
% At a switching surface it selects a neighboring branch for diagnostics.
% See OBSERVER_ISS_THEORY.md for the global incremental matrix-LMI proof.

    switch string(action)
        case "enclosure"
            result = localEnclosure(localDomain(varargin{1}));
        case "evaluate"
            result = localJacobian(varargin{1},varargin{2},localDomain(varargin{3}));
        otherwise
            error("nrmmTargetJacobian:invalidAction","Use 'enclosure' or 'evaluate'.");
    end
end

function values = localDomain(domain)
    minimum = domain.speedMinimum;
    if minimum == 0
        minimum = domain.regularizationSpeedMinimum;
    end
    values = [minimum,domain.speedMaximum,domain.scalarAccelerationMaximum, ...
        domain.yawRateMaximum,domain.accelerationNormBound];
    validateattributes(values,{'double'},{'real','finite','positive','size',[1,5]});
    if values(1) >= values(2)
        error("nrmmTargetJacobian:invalidDomain","The inverse floor must be below the maximum speed.");
    end
end

function enclosure = localEnclosure(domain)
    persistent cachedDomain cachedEnclosure
    if isequal(domain,cachedDomain)
        enclosure = cachedEnclosure;
        return
    end
    minimum = domain(1); maximum = domain(2);
    acceleration = domain(3); yaw = domain(4); cap = domain(5);
    subdivisions = 8;
    groups = cell(2*3*subdivisions^3*12,1);
    groupCount = 0;
    cellCount = 0;
    regionCells = zeros(3,1);
    for region = 1:3
        radial = linspace(0,1,subdivisions+1);
        lateral = linspace(0,cap,subdivisions+1);
        if region == 1
            radial = [0,1]; % The inner Jacobian uses raw clipped arguments.
        elseif region == 2
            radial = linspace(minimum,maximum,subdivisions+1);
            lateral = linspace(0,cap/minimum,subdivisions+1);
        end
        longitudinal = linspace(-cap,cap,subdivisions+1);
        for accelerationMode = -1:1
            for yawMode = 0:1
                for projected = 0:1
                    polynomial = localPolynomial(region,accelerationMode,yawMode,projected,domain);
                    for radialIndex = 1:numel(radial)-1
                        for longitudinalIndex = 1:subdivisions
                            for lateralIndex = 1:subdivisions
                                box = [radial(radialIndex:radialIndex+1); ...
                                    longitudinal(longitudinalIndex:longitudinalIndex+1); ...
                                    lateral(lateralIndex:lateralIndex+1)];
                                box = localIntersect(box,region,accelerationMode,yawMode, ...
                                    projected,minimum,maximum,acceleration,yaw,cap);
                                if isempty(box), continue; end
                                vertices = localBernstein(polynomial,box);
                                groupCount = groupCount+1;
                                groups{groupCount} = vertices;
                                cellCount = cellCount+1;
                                regionCells(region) = regionCells(region)+1;
                                if projected || region == 1
                                    % The projection derivative has a scalar factor in [0,1].
                                    % The inverse-floor interior has the common factor (r/a)^2.
                                    vertices(5:8,:) = 0;
                                    groupCount = groupCount+1;
                                    groups{groupCount} = vertices;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    vertices = unique([zeros(8,1),groups{1:groupCount}].','rows').';
    [~,lowerIndices] = min(vertices,[],2);
    [~,upperIndices] = max(vertices,[],2);
    zeroIndex = find(all(vertices==0,1),1);
    enclosure = struct("vertices",reshape(vertices,2,4,[]), ...
        "initialIndices",unique([zeroIndex;lowerIndices;upperIndices]), ...
        "domain",domain,"subdivisions",subdivisions,"cellCount",cellCount, ...
        "regionCellCounts",regionCells,"vertexCount",size(vertices,2), ...
        "construction","global rational-Bernstein vector-Jacobian enclosure", ...
        "scope","all states of the retained extension, up to orthogonal conjugation", ...
        "floatingPointVerified",false);
    cachedDomain = domain;
    cachedEnclosure = enclosure;
end

function box = localIntersect(box,region,accelerationMode,yawMode,projected,a,b,acceleration,yaw,cap)
    radial = box(1,:);
    accelerationScale = [1,1];
    yawScale = [1/a,1/a];
    if region == 2
        yawScale = [1,1];
    elseif region == 3
        accelerationScale = radial;
        yawScale = radial.^2/b;
    end
    if accelerationMode == 0 && accelerationScale(1) > 0
        box(2,:) = [max(box(2,1),-acceleration/accelerationScale(1)), ...
            min(box(2,2),acceleration/accelerationScale(1))];
    elseif accelerationMode == 1
        box(2,1) = max(box(2,1),acceleration/accelerationScale(2));
    elseif accelerationMode == -1
        box(2,2) = min(box(2,2),-acceleration/accelerationScale(2));
    end
    if yawMode == 0 && yawScale(1) > 0
        box(3,2) = min(box(3,2),yaw/yawScale(1));
    elseif yawMode == 1
        box(3,1) = max(box(3,1),yaw/yawScale(2));
    end
    if any(box(:,1)>box(:,2))
        box = [];
        return
    end
    minimumX = max([0,box(2,1),-box(2,2)]);
    radialScale = 1;
    if region == 2, radialScale = radial(1); end
    tolerance = 256*eps(max(1,cap^2));
    if minimumX^2+(radialScale*box(3,1))^2 > cap^2+tolerance
        box = [];
        return
    end
    xCap = sqrt(max(0,cap^2-(radialScale*box(3,1))^2));
    box(2,:) = [max(box(2,1),-xCap),min(box(2,2),xCap)];
    box(3,2) = min(box(3,2),sqrt(max(0,cap^2-minimumX^2))/radialScale);
    if region == 2, radialScale = radial(2); end
    if any(box(:,1)>box(:,2)) || (projected && region ~= 1 ...
            && max(box(2,:).^2)+(radialScale*box(3,2))^2 < cap^2-tolerance)
        box = [];
    end
end

function polynomial = localPolynomial(region,accelerationMode,yawMode,projected,domain)
% Compact variables: (unused,u,a*v), (r,u,v), or (b/r,x,y).
% The middle-region polynomial is multiplied by the positive denominator r^2.
    a = domain(1); b = domain(2); acceleration = domain(3); yaw = domain(4); cap = domain(5);
    zero = zeros(6,4,4); one = zero; one(1,1,1) = 1;
    t = zero; t(2,1,1) = 1;
    x = zero; x(1,2,1) = 1;
    y = zero; y(1,1,2) = 1;
    multiply = @localProduct;
    tSquared = multiply(t,t);
    u = x; v = y/a;
    if region == 2
        v = y;
    elseif region == 3
        u = multiply(t,x); v = multiply(tSquared,y)/b;
    end
    chi = double(accelerationMode==0); eta = double(yawMode==0);
    alpha = accelerationMode*acceleration*one;
    if chi, alpha = u; end
    nu = yaw*one;
    if eta, nu = v; end
    nuSquared = multiply(nu,nu);
    alphaNu = multiply(alpha,nu);
    alphaU = multiply(alpha,u);
    nuV = multiply(nu,v);
    denominator = one;
    if region == 1
        q = {-(1+2*eta)*nuSquared, multiply(nu,2*eta*u-3*alpha)/a; ...
            3*(1+chi+eta)*alphaNu/a, -nuSquared+3*chi*nuV-3*eta*alphaU/a^2};
        g = {zero,-2*eta*nu;3*chi*nu,3*eta*alpha/a};
    elseif region == 2
        denominator = tSquared;
        q = {(2*eta-1)*multiply(nuSquared,tSquared), multiply(multiply(nu,t),2*eta*u-3*alpha); ...
            -3*eta*multiply(alphaNu,t),multiply(-nuSquared+3*chi*nuV,tSquared)-3*eta*alphaU};
        g = {zero,-2*eta*multiply(nu,tSquared); ...
            3*chi*multiply(nu,tSquared),3*eta*multiply(alpha,t)};
    else
        q = {4*eta*multiply(t,nuSquared),multiply(multiply(nu,t),2*eta*multiply(t,u)-3*alpha)/b; ...
            -3*(chi+2*eta)*multiply(alphaNu,t)/b, ...
            -multiply(t,nuSquared)+3*chi*nuV-3*eta*multiply(alphaU,tSquared)/b^2};
        g = {zero,-2*eta*multiply(nu,tSquared); ...
            3*chi*multiply(nu,t),3*eta*multiply(alpha,tSquared)/b};
    end
    if projected
        projectedY = y;
        if region == 2, projectedY = multiply(t,y); end
        k = {multiply(projectedY,projectedY)/cap^2,-multiply(x,projectedY)/cap^2; ...
            -multiply(x,projectedY)/cap^2,multiply(x,x)/cap^2};
        product = cell(2);
        for row = 1:2
            for column = 1:2
                product{row,column} = multiply(g{row,1},k{1,column}) ...
                    +multiply(g{row,2},k{2,column});
            end
        end
        g = product;
    end
    entries = [q,g];
    coefficients = cat(4,entries{:},denominator);
    [first,second,third] = ind2sub([6,4,4],find(any(reshape(coefficients,96,9)~=0,2)));
    polynomial = coefficients(1:max(first),1:max(second),1:max(third),:);
end

function product = localProduct(first,second)
    full = convn(first,second);
    product = full(1:6,1:4,1:4);
end

function vertices = localBernstein(polynomial,box)
    coefficients = polynomial;
    for dimension = 1:3
        degree = size(polynomial,dimension)-1;
        lower = box(dimension,1); width = box(dimension,2)-lower;
        transform = zeros(degree+1);
        for row = 0:degree
            for power = 0:degree
                for index = 0:min(row,power)
                    transform(row+1,power+1) = transform(row+1,power+1) ...
                        +nchoosek(power,index)*lower^(power-index)*width^index ...
                            *nchoosek(row,index)/nchoosek(degree,index);
                end
            end
        end
        order = [dimension,setdiff(1:4,dimension,'stable')];
        permuted = permute(coefficients,order);
        shape = size(permuted,1:4);
        permuted = reshape(transform*reshape(permuted,shape(1),[]),shape);
        coefficients = ipermute(permuted,order);
    end
    coefficients = reshape(coefficients,[],9).';
    if any(coefficients(9,:)<=0) || any(~isfinite(coefficients),'all')
        error("nrmmTargetJacobian:invalidBernsteinDenominator", ...
            "The rational enclosure requires positive finite denominator coefficients.");
    end
    vertices = coefficients(1:8,:)./coefficients(9,:);
end

function jacobian = localJacobian(q,s,domain)
    validateattributes(q,{'double'},{'real','finite','size',[2,1]});
    validateattributes(s,{'double'},{'real','finite','size',[2,1]});
    r = norm(q);
    if r == 0
        jacobian = zeros(2,4);
        return
    end
    a = domain(1); b = domain(2); acceleration = domain(3); yaw = domain(4); cap = domain(5);
    cross = [0,-1;1,0];
    direction = q/r;
    denominator = max(r,a);
    denominatorDerivative = zeros(1,2);
    directionDerivative = eye(2)/a;
    if r >= a
        denominatorDerivative = direction.';
        directionDerivative = (eye(2)-direction*direction.')/r;
    end
    velocity = min(1,b/r)*q;
    velocityDerivative = eye(2);
    if r >= b
        velocityDerivative = (b/r)*(eye(2)-direction*direction.');
    end
    projected = min(1,cap/max(norm(s),realmin))*s;
    accelerationDerivative = eye(2);
    if norm(s) >= cap
        accelerationDerivative = (cap/norm(s))*(eye(2)-(s*s.')/norm(s)^2);
    end
    rawAlpha = velocity.'*projected/denominator;
    rawNu = (cross*velocity).'*projected/denominator^2;
    alpha = min(max(rawAlpha,-acceleration),acceleration);
    nu = min(max(rawNu,-yaw),yaw);
    alphaDerivative = double(abs(rawAlpha)<acceleration)*[ ...
        projected.'*velocityDerivative/denominator-rawAlpha*denominatorDerivative/denominator, ...
        velocity.'*accelerationDerivative/denominator];
    nuDerivative = double(abs(rawNu)<yaw)*[ ...
        projected.'*cross*velocityDerivative/denominator^2-2*rawNu*denominatorDerivative/denominator, ...
        (cross*velocity).'*accelerationDerivative/denominator^2];
    crossDirection = cross*q/denominator;
    jacobian = -nu^2*[velocityDerivative,zeros(2)] ...
        +(-2*nu*velocity+3*alpha*crossDirection)*nuDerivative ...
        +3*nu*crossDirection*alphaDerivative ...
        +3*alpha*nu*[cross*directionDerivative,zeros(2)];
end
