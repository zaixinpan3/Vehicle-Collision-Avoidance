function value = nrmmPositionVelocity(action,varargin)
%nrmmPositionVelocity Position-only, IMU-aided inertial velocity observer.
% The four numerical states are a position and a velocity. GNSS supplies
% position only. Each signed enclosure column multiplies an independent
% two-dimensional unit ball; signs are retained through the filter dynamics.
    switch string(action)
        case "initialize"
            value=localInitialize(varargin{:});
        case "measure"
            value=localMeasure(varargin{:});
        case "predict"
            value=localPredict(varargin{:});
        case "pointMeasure"
            value=localPointMeasure(varargin{:});
        case "pointPredict"
            value=localPointPredict(varargin{:});
        otherwise
            error('nrmmPositionVelocity:invalidAction','Unknown action.');
    end
end

function filter=localInitialize(position,velocity,positionRadius,velocityRadius,dt,bandwidth)
    validateattributes(position,{'double'},{'vector','numel',2,'real','finite'});
    validateattributes(velocity,{'double'},{'vector','numel',2,'real','finite'});
    validateattributes(dt,{'double'},{'scalar','finite','positive'});
    validateattributes(bandwidth,{'double'},{'scalar','finite','positive'});
    validateattributes(positionRadius,{'double'},{'scalar','finite','nonnegative'});
    validateattributes(velocityRadius,{'double'},{'scalar','finite','nonnegative'});
    pole=exp(-bandwidth*dt);
    % Both posterior error poles equal pole; beta has units 1/s.
    gain=[1-pole^2;(1-pole)^2/dt];
    filter=struct('point',[position(:);velocity(:)],'gain',gain, ...
        'generator',diag([positionRadius,velocityRadius]), ...
        'samplePeriod',dt,'bandwidth',bandwidth, ...
        'time',0,'historyTime',zeros(1,0),'historyPosition',zeros(2,0), ...
        'historyRadius',zeros(1,0),'historyAcceleration',zeros(1,0), ...
        'kind',"position-only-imu-velocity-v1");
end

function filter=localMeasure(filter,position,positionRadius)
    % Position secants are an independent, bounded-noise velocity enclosure.
    % They remain informative when a global intersample jerk bound is loose.
    % Retain actual GNSS samples; deterministic noise is never averaged down.
    keep=filter.historyTime<filter.time & filter.historyTime>=filter.time-4/filter.bandwidth;
    filter.historyTime=[filter.historyTime(keep),filter.time];
    filter.historyPosition=[filter.historyPosition(:,keep),position(:)];
    filter.historyRadius=[filter.historyRadius(keep),positionRadius];
    filter.historyAcceleration=[filter.historyAcceleration(keep),Inf];
    if isfield(filter,'initialPositionPending') && filter.initialPositionPending
        % The runtime's initial position is only a numerical guess. Anchor
        % this new filter to the first actual GNSS sample before correcting
        % velocity; a single position cannot supply a velocity observation.
        filter.point(1:2)=position(:);
        filter.generator(1,:)=0;
        filter.generator=[filter.generator,[positionRadius;0]];
        radius=sum(abs(filter.generator),2);
        filter.positionRadius=radius(1);filter.velocityRadius=radius(2);
        filter.initialPositionPending=false;
        return;
    end
    filter.point=localPointMeasure(filter.point,position,filter.gain);
    correction=eye(2)-filter.gain*[1,0];
    filter.generator=[correction*filter.generator,filter.gain*positionRadius];
    radius=sum(abs(filter.generator),2);
    filter.positionRadius=radius(1);filter.velocityRadius=radius(2);
    available=filter.velocityRadius;maximum=0;
    for k=numel(filter.historyTime)-1:-1:1
        maximum=max(maximum,filter.historyAcceleration(k));
        age=filter.time-filter.historyTime(k);
        center=(position(:)-filter.historyPosition(:,k))/age;
        bound=(positionRadius+filter.historyRadius(k))/age+maximum*age/2;
        available=min(available,norm(center-filter.point(3:4))+bound);
    end
    if available<filter.velocityRadius
        % An intersection cannot be represented by scaling old signed
        % columns. Keep the position projection and take its Cartesian
        % product with the new velocity ball; this encloses the intersection.
        filter.generator(2,:)=0;filter.generator=[filter.generator,[0;available]];
        filter.velocityRadius=available;
    end
end

function filter=localPredict(filter,acceleration,yaw,yawRadius,design,holdBounds)
    dt=filter.samplePeriod;
    filter.point=localPointPredict(filter.point,acceleration,yaw,dt);
    amax=holdBounds.acceleration;
    if ~isempty(filter.historyAcceleration),filter.historyAcceleration(end)=amax;end
    filter.time=filter.time+dt;
    % Compare the held world acceleration with the actual interval. IMU
    % noise, yaw uncertainty and intersample acceleration variation are all
    % included. Without a jerk envelope use the declared acceleration domain.
    mismatch=amax+norm(acceleration);
    jerk=holdBounds.bodyAccelerationRate+design.operatingDomain.egoYawRateMaximum*amax;
    localMismatch=design.sensors.accelerometerNoiseMaximum ...
        +2*norm(acceleration)*sin(min(pi,yawRadius)/2)+jerk*dt;
    mismatch=min(mismatch,localMismatch);
    transition=[1,dt;0,1];
    if isfinite(mismatch)
        % The two integrals of a varying disturbance need not share a sign.
        forcing=diag([dt^2/2,dt])*mismatch;
    else
        % Finite speed domain, with no unjustified intersample acceleration
        % premise. This remains an enclosure, although usually very loose.
        vmax=design.operatingDomain.egoSpeedMaximum;
        forcing=diag([2*vmax*dt+norm(acceleration)*dt^2/2, ...
            2*vmax+norm(acceleration)*dt]);
    end
    filter.generator=[transition*filter.generator,forcing];
end

function point=localPointMeasure(point,position,gain)
    residual=position(:)-point(1:2);
    point=point+[gain(1)*residual;gain(2)*residual];
end

function point=localPointPredict(point,acceleration,yaw,dt)
    rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    a=rotation*acceleration(:);
    point=[point(1:2)+dt*point(3:4)+dt^2*a/2;point(3:4)+dt*a];
end
