#!/usr/bin/env python
import roslib
import sys
import unittest


class TestBareBones(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('guidebot_gazebo', 'test_bare_bones', TestBareBones)
