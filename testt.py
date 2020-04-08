#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  7 19:44:02 2020

@author: thomaswierda
"""

import sys 
import os
import pathlib
current_directory = str(pathlib.Path().absolute())
sys.path.append(os.path.abspath(current_directory + "/ORTEC-challenge-master/helpClasses"))
import InstanceCVRPTWUI

def print_instance(instance):
    print("Dataset: ",instance.Dataset)
    print("Name: ",instance.Name)
    print("Days: ",instance.Days)
    print("Capacity: ",instance.Capacity)
    print("Max Distance: ",instance.MaxDistance)
    print("Depot Coordinate: ",instance.DepotCoordinate) # Dont fully understand this yet
    print("Vehicle Cost: ",instance.VehicleCost)
    print("Vehicle Day Cost: ", instance.VehicleDayCost)
    print("Distance Cost: ", instance.DistanceCost)

    print("\nTools: ",instance.Tools) # note that this prints Tool objects

    print("\nCoordinates: ", instance.Coordinates) # note that this prints Coordinate objects

    print("\nrequests: ", instance.Requests) # note that this prints Request objects

    instance.calculateDistances()
    print("\nDistances: ", instance.calcDistance)    

instance_file = "ORTEC-challenge-master/helpClasses/input/testInstance.txt"
instance = InstanceCVRPTWUI.InstanceCVRPTWUI(instance_file, "txt")

print_instance(instance)

instance.calculateDistances()
for i in range(len(instance.calcDistance)):
    print (i)

from collections import deque, namedtuple


# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class Graph:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2,3,4,5,6,7]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path

def create_graph(instance):
    nodes = []
    instance.calculateDistances()
    distances = instance.calcDistance
    for i in range(len(distances)):
        for j in range(len(distances)):
            if i == j:
                continue
            nodes.append((str(i),str(j),distances[i][j]))
        
    print(nodes)

graphie = create_graph(instance)    

graph = Graph(graphie)

print(graph.dijkstra("0", "2"))