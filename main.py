#!/usr/bin/env python
# -*- coding:utf-8 -*-
from Graph import Graph

if __name__ == '__main__':
        g = Graph()
        print("loading graph.json")
        g.load('graph.json')

        print("\n\nprint graph")
        g.printGraph()

        print("\n\nshow adjacence list")
        g.adjacenceList()

        print("\n\nfind ways from A to F")
        ways = g.findPaths('A', 'F')
        for way in ways:
                print(way)

        print("\n\nMST DFS")
        g.mstDFS('A')

        print("\n\nMST BFS")
        g.mstBFS('A')

        print("\n\nPRIM MST")
        g.prim('A')

        print("\n\nKRUSKAL MST")
        g.kruskal()

        print("\n\nDIJKSTRA SHORTEST PATH")
        g.dijkstra('A', 'F')