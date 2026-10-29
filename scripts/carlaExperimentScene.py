"""Create an empty-traffic CARLA fixture, retaining roads and buildings."""


def prepare_two_vehicle_scene(world):
    import carla
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    actors = world.get_actors()
    removed = {}
    for pattern in ('controller.ai.walker', 'walker.*', 'vehicle.*'):
        matches = actors.filter(pattern)
        removed[pattern] = sum(bool(actor.destroy()) for actor in matches)
    identifiers = set()
    for name in ('Vehicles', 'Car', 'Truck', 'Bus', 'Motorcycle', 'Bicycle', 'Train'):
        label = getattr(carla.CityObjectLabel, name, None)
        if label is not None:
            identifiers.update(obj.id for obj in world.get_environment_objects(label))
    if identifiers:
        world.enable_environment_objects(identifiers, False)
    # CARLA actor lists reflect the latest world snapshot. The caller checks
    # the final two-actor invariant after its setup tick.
    return {'unloadedParkedVehicles': True,
            'disabledVehicleEnvironmentObjects': len(identifiers),
            'removedDynamicVehicles': removed['vehicle.*'],
            'removedWalkers': removed['walker.*']}


def assert_only_experiment_vehicles(world, actors):
    expected = {actor.id for actor in actors}
    actual = {actor.id for actor in world.get_actors().filter('vehicle.*')}
    if len(expected) != 2 or actual != expected:
        raise RuntimeError(f'Expected only ego/target vehicle IDs {expected}; found {actual}')
