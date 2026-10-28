import itertools
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pnp_detect.geometry import order_corners,validate_quad,line_intersection


class GeometryTests(unittest.TestCase):
    def test_all_permutations(self):
        expected=np.array([[100,80],[400,110],[450,330],[70,300]],float)
        for permutation in itertools.permutations(expected.tolist()):
            np.testing.assert_allclose(order_corners(permutation),expected)

    def test_diamond_tie_deterministic(self):
        p=[[200,50],[350,200],[200,350],[50,200]]
        for perm in itertools.permutations(p):
            np.testing.assert_array_equal(order_corners(perm),p)

    def test_invalid_quads(self):
        examples=[[[0,0],[0,0],[30,30],[0,30]],
                  [[20,20],[40,40],[60,60],[80,80]],
                  [[50,50],[300,300],[300,50],[50,300]],
                  [[-1,40],[300,40],[300,300],[50,300]],
                  [[50,50],[300,50],[300,float('nan')],[50,300]],
                  [[50,50],[300,50],[100,100],[50,300]],
                  [[10,10],[12,10],[12,12],[10,12]]]
        for q in examples:
            with self.subTest(q=q):self.assertFalse(validate_quad(q,640,480)[0])

    def test_valid_perspective(self):
        self.assertTrue(validate_quad([[100,80],[400,110],[450,330],[70,300]],640,480)[0])

    def test_order_rejects_bad_input(self):
        for q in ([[1,1]]*4, [[1,1],[2,2],[3,3],[4,4]], [[1,1],[2,2],[float('inf'),3],[4,1]]):
            with self.assertRaises(ValueError):order_corners(q)

    def test_line_intersections(self):
        np.testing.assert_allclose(line_intersection(((0,1),(1,0)),((2,0),(0,1))),[2,1])
        with self.assertRaises(ValueError):line_intersection(((0,0),(1,0)),((0,1),(1,0)))


if __name__=='__main__':unittest.main()
