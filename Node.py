#!/usr/bin/env python
# -*- coding:utf-8 -*-

class Node(object):
        __key = None
        __edge = None
        __visited = False

        def __init__(self, key):
                self.__key = key
                self.__edge = []

        def getEdges(self):
                return self.__edge

        def getKey(self):
                return self.__key

        def addEdge(self, edge):
                self.__edge.append(edge)

        def visit(self):
                self.__visited = True

        def unvisit(self):
                self.__visited = False

        def isVisited(self):
                return self.__visited