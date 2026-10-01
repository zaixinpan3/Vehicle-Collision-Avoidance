function vehicle = mncavVehicleConfig()
% mncavVehicleConfig Nominal UMN MnCAV (2021 Chrysler Pacifica Hybrid) parameters.
%
% The values are a copy of config/mncavVehicleParameters.json of the Vehicle
% Localization repository (parameter review 2026-09-25, commit
% a1a73aad52a5d236e9936508bc763c861c38c0e8, file SHA-256
% 6d97bb165b4771f06292c65a34e7e58ecc79f8b5b1a0eaa357bf495b51a4dd21). That
% file records the sources, the identification and the sensitivity ranges;
% they are not repeated here.
%
% Mass and geometry describe the stock vehicle at curb weight, not the
% occupied, instrumented test vehicle: the loaded mass and center of gravity
% are unknown. The axle distances follow from the stock wheelbase and the
% static axle-load fractions. Yaw inertia and the cornering stiffnesses are
% fitted with mass and geometry fixed and are conditional on them.

    vehicle.identity = "UMN MnCAV: 2021 Chrysler Pacifica Hybrid";
    vehicle.mass = 2273.0;                          % kg, published curb mass
    vehicle.frontAxleDistance = 1.374605;           % m, center of gravity to front axle
    vehicle.rearAxleDistance = 1.714395;            % m, center of gravity to rear axle
    vehicle.wheelbase = 3.089;                      % m
    vehicle.length = 5.189;                         % m
    vehicle.width = 2.022;                          % m, without mirrors
    vehicle.yawInertia = 4485.72224015285;          % kg m^2, fitted
    % Complete-axle values, not per tire.
    vehicle.frontCorneringStiffness = 124178.82898505156; % N/rad, fitted
    vehicle.rearCorneringStiffness = 139550.58470228594;  % N/rad, fitted
    vehicle.steeringRatio = 16.2;                   % steering wheel / road wheel
end
