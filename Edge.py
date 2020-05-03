#!/usr/bin/env python
# -*- coding:utf-8 -*-

class Edge(object):
        __node_a = None
        __node_b = None
        __weight = None

        def __init__(self, a, b, w=1.0):
                self.__node_a = a
                self.__node_b = b
                self.__weight = w

        def getA(self):
                return self.__node_a

        def getB(self):
                return self.__node_b

        def getWeight(self):
                return self.__weight

        def setA(self, a):
                self.__node_a = a

        def setB(self, b):
                self.__node_b = b

        def setWeight(self, w):
                self.__weight = w
