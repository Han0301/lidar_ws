"""Synthetic metric correctness checks only; these are not robot test results."""
import unittest
import numpy as np
from common import describe, fit_plane, overlap, residual_metrics


class MetricsTest(unittest.TestCase):

    def test_known_plane_with_outliers(self):
        rng=np.random.default_rng(7)
        p=np.column_stack([rng.uniform(-2,2,(1000,2)),rng.normal(0,.01,1000)])
        p[:100,2]+=.25
        cfg=dict(seed=10,iterations=250,threshold_m=.05,min_points=100,max_fit_points=12000)
        n,d,s,m=fit_plane(p,cfg)
        self.assertGreater(abs(n[2]),.999)
        self.assertLess(abs(d),.005)
        self.assertEqual(int(m.sum()),900)
        self.assertAlmostEqual(residual_metrics(s,.05)['outlier_fraction'],.1)
        self.assertGreater(residual_metrics(s,.05)['rms_m'],.07)

    def test_rolling_origin_does_not_create_changes(self):
        a=dict(grid=np.array([[0,254,255,0],[0,0,254,0]],np.uint8),origin=[0,0],resolution=.1)
        b=dict(grid=np.array([[254,255,0,254],[0,254,0,255]],np.uint8),origin=[.1,0],resolution=.1)
        x,y=overlap(a,b,0)
        np.testing.assert_array_equal(x,y)

    def test_unaligned_origin_rejected(self):
        a=dict(grid=np.zeros((3,3)),origin=[0,0],resolution=.1)
        b=dict(grid=np.zeros((3,3)),origin=[.04,0],resolution=.1)
        with self.assertRaises(ValueError):
            overlap(a,b,0)

    def test_quantiles_and_missing_values(self):
        d=describe([1,2,3,4,np.nan])
        self.assertEqual(d['count'],4)
        self.assertEqual(d['p50'],2.5)
        self.assertAlmostEqual(d['p95'],3.85)
        self.assertIsNone(describe([])['max'])

    def test_degenerate_plane_rejected(self):
        cfg=dict(seed=10,iterations=20,threshold_m=.05,min_points=100,max_fit_points=12000)
        self.assertIsNone(fit_plane(np.zeros((150,3)),cfg))


if __name__=='__main__':
    unittest.main()
