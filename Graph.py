#!/usr/bin/env python
# -*- coding:utf-8 -*-
import json
from queue import Queue

from Edge import Edge
from Node import Node


class Graph(object):
        __nodes = None
        __ways = None

        def __init__(self):
                self.__nodes = {}
                self.__ways = []

        def load(self, file):
                f = open(file, 'r')
                data = f.read()
                f.close()
                j = json.loads(data)
                for entry in j['nodes']:
                        n = Node(entry['name'])
                        self.__nodes[entry['name']] = n

                for entry in j['nodes']:
                        n = self.__nodes[entry['name']]
                        for to in entry['to']:
                                e = Edge(
                                        a=self.__nodes[n.getKey()],
                                        b=self.__nodes[to['node']],
                                        w=to['weight']
                                )
                                n.addEdge(e)

        def printGraph(self):
                for key in sorted(self.__nodes.keys()):
                        print("Node", key)
                        for edge in self.__nodes[key].getEdges():
                                print(edge.getA().getKey(), "--->", edge.getB().getKey(), "weight:", edge.getWeight())

                        print("-" * 30)

        def adjacenceList(self):
                for key in sorted(self.__nodes.keys()):
                        entry = "[" + key + "]"
                        for edge in self.__nodes[key].getEdges():
                                entry += " ---> " + edge.getB().getKey()

                        print(entry)

        def getNode(self, key):
                if key in self.__nodes.keys():
                        return self.__nodes[key]

        def unvisitAllNodes(self):
                for n in self.__nodes.values():
                        n.unvisit()

        def allVisited(self):
                f = []
                for n in self.__nodes.values():
                        f.append(n.isVisited())

                return False in f

        def findPaths(self, a, b):
                self.unvisitAllNodes()
                visited = {}
                for n in self.__nodes.values():
                        visited[n.getKey()] = False

                pathList = [a]
                self.__findPaths(a, b, visited, pathList)
                ways = self.__ways
                self.__ways = []
                return ways

        def __findPaths(self, src, dst, visited, path_list):
                visited[src] = True

                if src == dst:
                        # print(path_list)
                        self.__ways.append(' ---> '.join(path_list))
                        visited[src] = False
                else:
                        nodes = []
                        joinable = False
                        for edge in self.__nodes[src].getEdges():
                                if edge.getA().getKey() == src:
                                        joinable = True
                                        nodes.append(edge.getB())

                        for node in nodes:
                                if not visited[node.getKey()] and joinable:
                                        path_list.append(node.getKey())
                                        self.__findPaths(node.getKey(), dst, visited, path_list)
                                        path_list.remove(node.getKey())

                        visited[src] = False
                        nodes.clear()

        def mstDFS(self, key):
                self.unvisitAllNodes()
                self.__mstDFS(key)

        def __mstDFS(self, key):
                n = self.getNode(key)
                n.visit()

                for edge in n.getEdges():
                        if not edge.getB().isVisited():
                                print(edge.getA().getKey(), "--->", edge.getB().getKey(), "weight:", edge.getWeight())
                                self.__mstDFS(edge.getB().getKey())

        def mstBFS(self, key):
                self.unvisitAllNodes()
                q = Queue()
                start = self.getNode(key)
                q.put(start)
                start.visit()
                while not q.empty():
                        next = q.get()
                        print("Visited ", next.getKey())
                        for edge in next.getEdges():
                                if not edge.getB().isVisited():
                                        print(edge.getA().getKey(), "--->", edge.getB().getKey(), "weight:",
                                              edge.getWeight())
                                        q.put(edge.getB())
                                        edge.getB().visit()

        def __min(self, q):
                _min = None
                for entry in q:
                        if _min == None:
                                _min = entry

                        elif entry[1] < _min[1]:
                                _min = entry

                return _min

        def prim(self, key):
                cost = 0.0
                mst = []
                self.unvisitAllNodes()
                n = self.getNode(key)
                q = [(n, 0)]
                n.visit()
                while len(q) != 0:
                        entry = self.__min(q)
                        q.remove(entry)
                        current = entry[0]
                        current.visit()

                        # print("Current node", current.getKey(), 'Weight:', entry[1])

                        for edge in current.getEdges():
                                if not edge.getB().isVisited():
                                        edge.getB().visit()
                                        print(current.getKey(), "--->", edge.getB().getKey(), "weight:",
                                              edge.getWeight())
                                        mst.append([current.getKey(), edge.getWeight(), edge.getB().getKey()])
                                        q.append((edge.getB(), edge.getWeight()))
                                        cost += edge.getWeight()

                print("MST:", mst)
                print("Total cost", cost)

        def __sort(self, edges):
                temp = []
                while len(edges) != 0:
                        e = self.__min(edges)
                        temp.append(e)
                        edges.remove(e)

                return temp

        def kruskal(self):
                cost = 0.0
                edges = []
                mst = []
                self.unvisitAllNodes()

                for node in self.__nodes.values():
                        for edge in node.getEdges():
                                edges.append((edge, edge.getWeight()))

                edges = self.__sort(edges)

                print("Running Kruskal")
                while len(edges) != 0:
                        e = edges.pop(0)
                        A = e[0].getA()
                        B = e[0].getB()

                        if not e in mst:
                                loop = False
                                for edge in B.getEdges():
                                        if edge.getB().isVisited():
                                                # print("LOOP", edge.getA().getKey(), "--->", edge.getB().getKey(), "weight:", edge.getWeight())
                                                loop = True

                                if not loop:
                                        mst.append([A.getKey(), e[0].getWeight(), B.getKey()])
                                        print(e[0].getA().getKey(), "--->", e[0].getB().getKey(), "weight:",
                                              e[0].getWeight())
                                        cost += e[1]
                                        B.visit()

                print("MST:", mst)
                print("Total cost", cost)

        def __minDistance(self, paths, include_shortest_path):
                """
                int min = INT_MAX, min_index;

   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
                """
                min = 2 ** 32
                i = None
                for node in self.__nodes.values():
                        if include_shortest_path[node.getKey()] == False and paths[node.getKey()] <= min:
                                min = paths[node.getKey()]
                                i = node

                return i

        def __connected(self, A, B):
                node = self.getNode(A)
                for edge in node.getEdges():
                        if edge.getB().getKey() == B:
                                return True

                return False

        def __getWeightBetween(self, A, B):
                node = self.getNode(A)
                for edge in node.getEdges():
                        if edge.getB().getKey() == B:
                                return edge.getWeight()

                return 2 ** 32

        def dijkstra(self, start, end):
                paths = self.__dijkstra(start)
                print(paths[end])

        def __dijkstra(self, start):
                paths = {}
                include_shortest_path = {}

                for node in self.__nodes.values():
                        paths[node.getKey()] = 2 ** 32
                        include_shortest_path[node.getKey()] = False

                paths[start] = 0

                for count in self.__nodes.values():
                        current = self.__minDistance(paths, include_shortest_path)
                        include_shortest_path[current.getKey()] = True
                        for node in self.__nodes.values():
                                if (
                                        include_shortest_path[node.getKey()] == False and
                                        self.__connected(current.getKey(), node.getKey()) and
                                        paths[current.getKey()] != 2 ** 32 and
                                        paths[current.getKey()] + self.__getWeightBetween(current.getKey(), node.getKey()) < paths[node.getKey()]
                                ):
                                        paths[node.getKey()] = paths[current.getKey()] + self.__getWeightBetween(current.getKey(), node.getKey())

                print(paths)
                print(include_shortest_path)
                return paths
