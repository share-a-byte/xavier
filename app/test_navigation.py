"""Run with: .venv/bin/python -m unittest app.test_navigation"""
import unittest

from . import navigation as nav


def sweep(name, x, z, floor_height=0.0, sees=(), floor=0):
    return {"id": name, "x": x, "y": floor_height + 1.35, "z": z, "puckY": floor_height, "floor": floor, "neighbors": list(sees)}


def label(a, b, kind, **extra):
    return {"from": a, "to": b, "type": kind, "wheelchairAccessible": kind in nav.DRIVABLE, **extra}


def building(labels=()):
    """A corridor A-B-C-D along +x with a second way round from B to C through E (north of the corridor),
    and stairs from A up a landing L to U, which stands above D with V beside it. Everything on the ground floor
    sees everything else there."""
    ground = {"A": (0, 0), "B": (4, 0), "C": (8, 0), "D": (12, 0), "E": (6, -5)}
    sweeps = [sweep(n, x, z, sees=[m for m in ground if m != n] + ["L", "U"]) for n, (x, z) in ground.items()]
    sweeps.append(sweep("L", -3, 0, 1.5, sees=[*ground, "U"]))
    sweeps.append(sweep("U", 12, 0.5, 3.0, sees=[*ground, "L", "V"], floor=1))
    sweeps.append(sweep("V", 15, 0.5, 3.0, sees=["U"], floor=1))
    return nav.Graph("test0000000", {"sweeps": sweeps, "floors": []}, {"edges": list(labels)})


CORRIDOR = [label("A", "B", "corridor"), label("B", "C", "corridor"), label("C", "D", "corridor"),
            label("B", "E", "corridor"), label("E", "C", "doorway", widthCm=80)]


class Thinning(unittest.TestCase):
    def test_long_sight_lines_give_way_to_the_scans_between(self):
        graph = building()
        self.assertIn("A~B", graph.edges)
        self.assertNotIn("A~C", graph.edges)
        self.assertNotIn("A~D", graph.edges)

    def test_a_scan_overhead_does_not_stand_in_for_the_floor_below(self):
        above = [sweep("A", 0, 0, sees=["B", "X"]), sweep("B", 6, 0, sees=["A", "X"]), sweep("X", 3, 0, 2.0, sees=["A", "B"])]
        graph = nav.Graph("test0000000", {"sweeps": above}, {})
        self.assertEqual(graph.edges["A~B"]["type"], "unknown")

    def test_height_changes_are_never_drivable_until_someone_says_so(self):
        graph = building()
        self.assertEqual(graph.edges["A~L"]["type"], "stairs")
        self.assertEqual(graph.edges["D~U"]["type"], "blocked")
        self.assertFalse(any(e["accessible"] for e in graph.edges.values()))


class Planning(unittest.TestCase):
    def test_unknown_level_hops_are_driven_by_astar(self):
        # No hand labels: A* still finds a path on Matterport level hops; stairs stay excluded.
        graph = building()
        result = nav.route(graph, "A", "D")
        self.assertTrue(result["found"])
        self.assertEqual([w["id"] for w in result["waypoints"]], ["A", "B", "C", "D"])
        self.assertEqual(result["unverifiedLegs"], 3)
        self.assertFalse(nav.route(graph, "A", "U")["found"])

    def test_verified_route_and_its_legs(self):
        result = nav.route(building(CORRIDOR), "A", "D")
        self.assertEqual([w["id"] for w in result["waypoints"]], ["A", "B", "C", "D"])
        self.assertEqual(result["lengthM"], 12.0)
        self.assertEqual([leg["headingDeg"] for leg in result["legs"]], [0.0, 0.0, 0.0])
        self.assertEqual(result["unverifiedLegs"], 0)

    def test_turns_are_positive_to_the_left(self):
        # E is at smaller z than the corridor, which is to the left of someone driving along +x.
        result = nav.route(building(CORRIDOR), "A", "E")
        self.assertEqual([w["id"] for w in result["waypoints"]], ["A", "B", "E"])
        self.assertGreater(result["legs"][1]["turnDeg"], 0)
        self.assertAlmostEqual(result["legs"][1]["headingDeg"], 68.2, places=1)
        facing_back = nav.route(building(CORRIDOR), "A", "E", heading_deg=180)
        self.assertEqual(abs(facing_back["legs"][0]["turnDeg"]), 180.0)

    def test_stairs_are_walked_but_not_driven(self):
        graph = building(CORRIDOR + [label("L", "U", "stairs")])
        self.assertFalse(nav.route(graph, "A", "U")["found"])
        self.assertFalse(nav.route(graph, "A", "U", preview=True)["found"])
        self.assertEqual([w["id"] for w in nav.walking(graph, "A", "U")["waypoints"]], ["A", "L", "U"])

    def test_elevator_joins_floors_and_can_be_switched_off(self):
        graph = building(CORRIDOR + [label("D", "U", "elevator"), label("U", "V", "corridor")])
        result = nav.route(graph, "A", "V")
        self.assertTrue(result["usesElevator"])
        self.assertEqual(result["legs"][-2]["distanceM"], 0.0)
        self.assertEqual(result["lengthM"], 15.0)
        # Nobody knows which way the chair faces when it rolls out, so there is no turn to give.
        self.assertIsNone(result["legs"][-1]["turnDeg"])
        self.assertEqual(result["legs"][-1]["headingDeg"], 0.0)
        self.assertFalse(nav.route(graph, "A", "V", elevators=False)["found"])

    def test_narrow_doorway_and_hand_edited_flag(self):
        graph = building(CORRIDOR)
        self.assertEqual(len(nav.route(graph, "B", "C", blocked=["B~C"], min_width_cm=75)["waypoints"]), 3)
        self.assertFalse(nav.route(graph, "B", "C", blocked=["B~C"], min_width_cm=85)["found"])
        refused = building([{**label("A", "B", "corridor"), "wheelchairAccessible": False}])
        self.assertFalse(nav.route(refused, "A", "B")["found"])

    def test_places_and_scan_numbers_resolve(self):
        graph = nav.Graph("test0000000", {"sweeps": [sweep("A", 0, 0, sees=["B"]), sweep("B", 2, 0, sees=["A"])]},
                          {"destinations": [{"id": "lab", "name": "Robotics Lab", "sweep": "B"}]})
        self.assertEqual(graph.resolve("robotics lab"), "B")
        self.assertEqual(graph.resolve("1"), "A")
        with self.assertRaises(ValueError):
            graph.resolve("nowhere")


class Trips(unittest.TestCase):
    def test_blocked_hop_is_planned_around(self):
        graph = building(CORRIDOR)
        trip = nav.Trip(graph, "A", "D")
        self.assertEqual(trip.status, "planned")
        trip.report_pose({"x": 5.0, "z": 0.0, "headingDeg": 0, "waypoint": 2, "status": "blocked"})
        again = trip.report_blocked(graph, "B", "C")
        self.assertEqual([w["id"] for w in again["waypoints"]], ["B", "E", "C", "D"])
        self.assertEqual(trip.status, "replanned")
        self.assertEqual(trip.revision, 2)
        # The chair was facing along +x, so the new first leg starts with a left turn.
        self.assertGreater(again["legs"][0]["turnDeg"], 0)

    def test_with_no_way_round_the_chair_stays_stopped(self):
        graph = building(CORRIDOR)
        trip = nav.Trip(graph, "A", "D")
        trip.report_blocked(graph, "C", "D")
        self.assertFalse(trip.route["found"])
        self.assertEqual(trip.status, "no-route")

    def test_odometry_is_measured_from_the_start_facing_the_first_waypoint(self):
        trip = nav.Trip(building(CORRIDOR), "B", "E")  # first leg heads up and to the left of +x
        pose = trip.report_pose({"frame": "odom", "forwardM": 5.385, "leftM": 0.0, "turnDeg": 0})
        self.assertAlmostEqual(pose["x"], 6.0, places=2)
        self.assertAlmostEqual(pose["z"], -5.0, places=2)
        side = nav.Trip(building(CORRIDOR), "A", "D").report_pose({"frame": "odom", "forwardM": 1.0, "leftM": 0.5, "turnDeg": 90})
        self.assertEqual((side["x"], side["z"], side["headingDeg"]), (1.0, -0.5, 90.0))

    def test_bad_reports_are_refused(self):
        trip = nav.Trip(building(CORRIDOR), "A", "D")
        for pose in ({"x": float("nan"), "z": 0}, {"x": 0, "z": 0, "status": "flying"}, {"x": 0, "z": 0, "waypoint": 9},
                     {"frame": "moon", "x": 0, "z": 0}):
            with self.assertRaises(ValueError):
                trip.report_pose(pose)


if __name__ == "__main__":
    unittest.main()
