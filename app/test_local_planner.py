"""Run with: .venv/bin/python -m unittest app.test_local_planner app.test_navigation"""
import unittest

from . import local_planner as local
from . import navigation as nav
from .test_navigation import CORRIDOR, building, label


class LocalPlanner(unittest.TestCase):
    def test_direct_when_clear(self):
        result = local.plan((0, 0), (3, 0), [], chair_width_cm=70)
        self.assertTrue(result["found"])
        self.assertEqual(result["mode"], "direct")
        self.assertEqual(len(result["points"]), 2)

    def test_detours_around_a_circle(self):
        obstacle = {"x": 1.5, "z": 0.0, "radiusM": 0.5}
        result = local.plan((0, 0), (3, 0), [obstacle], chair_width_cm=70)
        self.assertTrue(result["found"], result.get("reason"))
        self.assertEqual(result["mode"], "detour")
        # Path must go around: some point has |z| well clear of the obstacle.
        self.assertTrue(any(abs(p["z"]) > 0.4 for p in result["points"]))
        self.assertFalse(local.blocks_segment(
            (result["points"][0]["x"], result["points"][0]["z"]),
            (result["points"][1]["x"], result["points"][1]["z"]),
            [obstacle], 70) or len(result["points"]) < 2)

    def test_blocked_hallway_asks_for_global_replan(self):
        # Solid box taller than the local search pad so there is no way around.
        wall = [{"xmin": 1.2, "xmax": 1.8, "zmin": -20.0, "zmax": 20.0}]
        result = local.plan((0, 0), (3, 0), wall, chair_width_cm=70)
        self.assertFalse(result["found"])
        self.assertRegex(result["reason"].lower(), r"blocked|no collision-free|no free space|too large")

    def test_goal_inside_obstacle(self):
        result = local.plan((0, 0), (1.5, 0), [{"x": 1.5, "z": 0, "radiusM": 0.6}], 70)
        self.assertFalse(result["found"])
        self.assertIn("waypoint", result["reason"].lower())

    def test_wider_chair_needs_more_space(self):
        obstacle = {"x": 1.5, "z": 0.55, "radiusM": 0.35}
        narrow = local.plan((0, 0), (3, 0), [obstacle], chair_width_cm=50)
        wide = local.plan((0, 0), (3, 0), [obstacle], chair_width_cm=90)
        self.assertTrue(narrow["found"])
        # Wide chair may still find a path further out, or fail — either way radius grows.
        self.assertGreater(local.chair_radius_m(90), local.chair_radius_m(50))


class ReplanOnWidth(unittest.TestCase):
    def test_narrow_door_rejects_wide_chair_and_stops(self):
        # Only ways A→D use 75 cm or 80 cm doorways — too narrow for a 90 cm chair.
        labels = [label("A", "B", "corridor"), label("B", "C", "doorway", widthCm=75),
                  label("C", "D", "corridor"), label("B", "E", "corridor"),
                  label("E", "C", "doorway", widthCm=80)]
        graph = building(labels)
        trip = nav.Trip(graph, "A", "D", min_width_cm=70)
        self.assertTrue(trip.route["found"])
        trip.set_width(graph, 90)
        self.assertFalse(trip.route["found"])
        self.assertEqual(trip.status, "no-route")
        self.assertIn("narrower", trip.route["reason"].lower())

    def test_failed_block_replan_never_resumes(self):
        graph = building(CORRIDOR)
        trip = nav.Trip(graph, "A", "D")
        trip.report_blocked(graph, "C", "D")
        self.assertEqual(trip.status, "no-route")
        self.assertFalse(trip.route["found"])
        # A second blocked report must not flip status back to following.
        trip.report_blocked(graph, "B", "C")
        self.assertEqual(trip.status, "no-route")


if __name__ == "__main__":
    unittest.main()
