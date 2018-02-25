#!/usr/bin/env python

import roslib

import sys
import unittest

class TestBareBones(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_woop(self):
        self.assertTrue(False)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('guidebot_gazebo', 'test_bare_bones', TestBareBones)
