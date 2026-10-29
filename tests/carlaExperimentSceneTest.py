"""Behavioral checks for isolated two-vehicle CARLA experiments."""
import fnmatch
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from carlaExperimentScene import prepare_two_vehicle_scene, assert_only_experiment_vehicles


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if fnmatch.fnmatch(a.type_id, pattern))


class World:
    def __init__(self):
        self.actors = ActorList()
        self.disabled = set()
        self.unloaded = []

    def add(self, identifier, kind):
        world = self
        class Actor:
            id = identifier
            type_id = kind
            def destroy(self):
                world.actors.remove(self)
                return True
        actor = Actor()
        self.actors.append(actor)
        return actor

    def get_actors(self):
        return ActorList(self.actors)

    def unload_map_layer(self, layer):
        self.unloaded.append(layer)

    def get_environment_objects(self, label):
        return [SimpleNamespace(id=i) for i in {'Car': [10, 11], 'Bus': [12], 'Buildings': [99]}.get(label, [])]

    def enable_environment_objects(self, identifiers, enabled):
        assert not enabled
        self.disabled.update(identifiers)


class CarlaExperimentSceneTest(unittest.TestCase):
    def test_clears_traffic_walkers_and_static_cars_without_hiding_buildings(self):
        world = World()
        world.add(1, 'vehicle.npc')
        world.add(2, 'walker.pedestrian')
        world.add(3, 'controller.ai.walker')
        api = SimpleNamespace(MapLayer=SimpleNamespace(ParkedVehicles='parked'),
                              CityObjectLabel=SimpleNamespace(Car='Car', Bus='Bus'))
        with patch.dict(sys.modules, {'carla': api}):
            result = prepare_two_vehicle_scene(world)
        self.assertEqual(world.actors, [])
        self.assertEqual(world.disabled, {10, 11, 12})
        self.assertEqual(world.unloaded, ['parked'])
        self.assertEqual(result['removedDynamicVehicles'], 1)
        self.assertEqual(result['disabledVehicleEnvironmentObjects'], 3)

    def test_accepts_exactly_the_two_owned_vehicles(self):
        world = World()
        actors = [world.add(1, 'vehicle.ego'), world.add(2, 'vehicle.target')]
        assert_only_experiment_vehicles(world, actors)
        world.add(3, 'vehicle.unexpected')
        with self.assertRaises(RuntimeError):
            assert_only_experiment_vehicles(world, actors)

    def test_rejects_missing_target(self):
        world = World()
        ego = world.add(1, 'vehicle.ego')
        with self.assertRaises(RuntimeError):
            assert_only_experiment_vehicles(world, [ego])


if __name__ == '__main__':
    unittest.main()
