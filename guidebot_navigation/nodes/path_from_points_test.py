#!/usr/bin/env python
import roslib
import sys
import unittest

from path_from_points import addido


class TestPathPositions(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('guidebot_navigation',
                    'test_path_from_points', TestPathFromPoints)
