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
        'kind',"position-only-imu-velocity-v1");
end

function filter=localMeasure(filter,position,positionRadius)
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
end

function filter=localPredict(filter,acceleration,yaw,yawRadius,design,holdBounds)
    dt=filter.samplePeriod;
    filter.point=localPointPredict(filter.point,acceleration,yaw,dt);
    amax=holdBounds.acceleration;
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
