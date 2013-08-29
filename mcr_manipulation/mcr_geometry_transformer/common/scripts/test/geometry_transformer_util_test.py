#!/usr/bin/env python

import unittest
import geometry_transformer_util.geometry_transformer_util


class TestWrenchTransformer(unittest.TestCase):

    def setUp(self):
        pass

    def test_fail(self):
        self.assertEquals(None, geometry_transformer_util.geometry_transformer_util.transform_wrench(None, None))


if __name__ == '__main__':
    unittest.main()