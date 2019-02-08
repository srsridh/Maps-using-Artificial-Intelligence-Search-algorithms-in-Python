#!/usr/bin/env python2.7

"""
1)

State Space - is the set of all cities given in our dataset
Successor Function - It is the immediate neighbor to our current city
Edge weights - It is the cost function
Start State - Is the start city from which the journey begins
Goal State - Is the city that is to be reached from our start state

Heuristic formula:
The Haversine formula(Reference -  MIT license) was used to compute the heuristic between two cities based on latitude and longitude.
dlon = longitude_two - longitude_one
dlat = latitude_two - latitude_one
a = sin(dlat / 2) ** 2 + cos(latitude_one) * cos(latitude_two) * sin(dlon / 2) ** 2
dist = 2 * atan2(sqrt(a), sqrt(1 - a))
dist = dist * 6373 * 0.621371
Where 6373*0.621371 is the radius of the earth in miles.

Distance(Heuristic) - The minimum distance calculated from Latitude and Longitude
Time(Heuristic) - Time is calcuated by Heuristic distance/ Maximum Speed

Heuristic
            |   BFS     DFS     IDS     Astar       Uniform
---------------------------------------------------------------------------------
            |
segment     |   0       0       0       0           0
            |
distance    |   0       0       0       distance    0
            |
time        |   0       0       0       time        0
Optimality
            |   BFS     DFS     IDS     Astar   Uniform
----------------------------------------------------------
            |
segment     |   yes     no      yes     yes     yes
            |
distance    |   no      no      no      yes     yes
            |
time        |   no      no      no      yes     yes

Our heuristics are admissable because:
    Heuristic as 0:
    When the heuristic is taken to be 0, it is always lesser than the actual distance to the goal state
    Heuristic as distance:
    When the heuristic is taken as the distance computed from latitude and longitude, it is the shortest possible distance measurement and is always going to be lesser or
    equal to the actual road distance to the goal state
    Heuristic as time:
    Time heuristic is computed as the minimum distance between two states to maximum speed. This means we will cover the minimum distance in maximum speed and the time
    consumed would be less than the actual time taken to reach our goal state

(2)
a)We have implemented astar, dfs, ids, bfs and uniform using slightly different variartions of search algorithm 3
b)The search algorithm pops out a given city based on what the algorithm is.
    *For astar and uniform we have used a priority queue and it pops out the highest priority element.
    *For BFS we used a FIFO queue
c)If that is the goal state, it returns.
d)We then find all the reachable successors(neighbouring cities) from this state and traverse through all of those.
e)We then compute the distance, time and cost to reach the successor state from the previous state. We also calculate the heuristic
to reach the goal state from this state. Then, we add the successor state into the fringe.
f)So, when we arrive at the goal state, the whole class with the goal city, its parent cities, distance and time taken to arrive
are computed


3) Challenges faced:
                    *By using Pandas Data frame to fetch latitude, longitude values for heuristic and compute time and distance
                     for costs, the program became extremly slow. For certain test cases like traversing from 'Lynnwood,_Washington' to
                    'Littlerock,_California',the code for astar was taking more than 60 seconds. Upon spending several hours,
                     we were able to understand that Pandas were consuming lot of memory and each computation was taking lot of time.
                     So, we decided to have a dictioanry with cities as key and the corresponding length and time taken as values
                     and another dictionary with similarly City as key and latitude and longitude as values, executed the code for the same
                     cities in 2.4 seconds. Which is a drastic change

Assumptions : For cities that did not have speed values, we had replaced it with the average speed across all cities.
              For cities that did not have Latitude and Longitude, we have assumed their heuristic to be zero
              For junctions since the geo-cordinates were not given, we have assumed 0 as heuristic
              The max speed is assumed for calculating time heuristic is taken to be the maximum of all the speeds in the dataset

Simplifications and Design decisions : We understood that the difference between dfs, bfs, ids, astar and uniform were the use of
"different type of queues", which simplified our code to a great extent.
"""

from __future__ import print_function
import pandas as pd
from math import sin, cos, sqrt, atan2, radians
import Queue as q
import numpy as np
import sys


# Computing Distance between cities based on Latitude and Longitude. Gets called from getHeuristic Function
def distanceBetweenCities(from_city, to_city):
    try:
        latitude_Longitude1 = latitude_Longitudedict[from_city]
        latitude_Longitude2 = latitude_Longitudedict[to_city]
        latitude_one = radians(latitude_Longitude1[0])
        longitude_one = radians(latitude_Longitude1[1])
        latitude_two = radians(latitude_Longitude2[0])
        longitude_two = radians(latitude_Longitude2[1])
        dlon = longitude_two - longitude_one
        dlat = latitude_two - latitude_one
        a = sin(dlat / 2) ** 2 + cos(latitude_one) * cos(latitude_two) * sin(dlon / 2) ** 2
        dist = 2 * atan2(sqrt(a), sqrt(1 - a))
        dist = dist * 6373 * 0.621371
        return int(dist)
    except IndexError:
        return 0


# Compute distance, time as heuristic values
def getHeuristic(current_state, goal_state, cost_function):
    cost = 0
    try:
        if routing_algo != 'astar':
            raise KeyError
        distance = distanceBetweenCities(current_state, goal_state)
        maxSpeed = max(road_segments.speed)
        time = distance / maxSpeed
        if (cost_function == 'distance'):
            cost = distance
        elif (cost_function == 'time'):
            cost = time
        return 0, distance, time, cost
    except KeyError:
        return 0, 0, 0, 0


# Computes the cost(distance,time) of travelling from one city to another\
def getCostToSuccessor(from_city, to_city, cost_function, city_Pair):
    length_Time = length_Timedict[city_Pair]
    dist = length_Time[0]
    time = length_Time[1]

    cost = 1
    if routing_algo == 'astar' or routing_algo == 'uniform':
        if cost_function == 'distance':
            cost = dist
        if cost_function == 'time':
            cost = time

    return 1, dist, time, cost


# Creating a class to store the cities, their parents and the costs and heuristics associated with it
class State(object):
    def __init__(self, cityName, segment, distance, time, heruristicCost, parentCityState, cost_function):
        self.cityName = cityName
        self.segment = segment
        self.distance = distance
        self.time = time
        self.parentCityState = parentCityState
        self.heuristicCost = heruristicCost
        if routing_algo == 'astar' or routing_algo == 'uniform':
            if cost_function == 'distance':
                self.cost = self.distance
            if cost_function == 'time':
                self.cost = self.time
            if cost_function == 'segments':
                self.cost = self.segment
        else:
            self.cost = self.segment

    def __lt__(self, other):
        return self.cost + self.heuristicCost < other.cost + other.heuristicCost


# Function to solve our algorithms.
def solve(road_segments, idsDepth=sys.maxsize):
    closed_list = []
    if initial_state == goal_state:
        return initial_state

    if routing_algo == 'bfs':
        fringe = q.Queue()
    elif routing_algo == 'dfs' or routing_algo == 'ids':
        fringe = q.LifoQueue()
    else:
        fringe = q.PriorityQueue()

    heurSegment, heurDistance, heurTime, heurCost = getHeuristic(initial_state, goal_state, cost_function)

    # dic of all the elements in fringe and explored elements
    previoslyExists = {}

    # heurCost is to return the heuristic as distance/time when inital and goal state are same.
    fringe.put(State(initial_state, 0, 0, 0, heurCost, None, cost_function))
    previoslyExists[(initial_state)] = 1

    while not fringe.empty():
        best = fringe.get()

        closed_list.append(best.cityName)
        if best.cityName == goal_state:
            return best

        if routing_algo == 'ids' and best.segment > idsDepth:
            continue

        if (best.cityName in adjacency_list.keys()):
            successor_list = adjacency_list[best.cityName]
            for i in successor_list:
                if i not in closed_list:
                    from_city = best.cityName
                    to_city = i
                    city_Pair = (from_city, to_city)
                    costToCurrent = best.cost
                    segmentSucc, distanceSucc, timeSucc, costSucc = getCostToSuccessor(from_city, to_city,
                                                                                       cost_function, city_Pair)
                    segmentHeur, distanceHeur, timeHeur, costHeur = getHeuristic(to_city, goal_state, cost_function)

                    # costSucc is to generalize. to avoid any loop. This will automatically use this value dependent on cost function
                    totalCostToSuccessor = costToCurrent + costSucc

                    currentHash = (to_city)
                    flag = False
                    if currentHash in previoslyExists:
                        for succ in list(fringe.queue):
                            if to_city in succ.cityName:
                                if succ.cost > totalCostToSuccessor:
                                    fringe.queue.remove(succ)
                                else:
                                    flag = True
                    if not flag:
                        fringe.put(
                            State(to_city, best.segment + segmentSucc, best.distance + distanceSucc,
                                  best.time + timeSucc,
                                  costHeur, best, cost_function))
                        previoslyExists[currentHash] = 1

    return False


# Accepting input from user
initial_state = sys.argv[1]
initial_state = initial_state.replace('"', '')

goal_state = sys.argv[2]
goal_state = goal_state.replace('"', '')

routing_algo = sys.argv[3]
cost_function = sys.argv[4]

# Dictionary with city as key and it's neighbors as values
if __name__ == '__main__':
    file_name = "road-segments.txt"
    adjacency_list = dict()
    with open('road-segments.txt', 'r') as fileRead:
        for line in fileRead:
            lineSplit = line.split(' ')
            if (lineSplit[0] not in adjacency_list):
                adjacency_list[lineSplit[0].replace('"', '')] = [lineSplit[1].replace('"', '')]
            else:
                adjacency_list[lineSplit[0].replace('"', '')].append(lineSplit[1].replace('"', ''))
            if (lineSplit[1] not in adjacency_list):
                adjacency_list[lineSplit[1].replace('"', '')] = [lineSplit[0].replace('"', '')]
            else:
                adjacency_list[lineSplit[1].replace('"', '')].append(lineSplit[0].replace('"', ''))

# Loading input text file into Pandas dataframe
city_gps = pd.read_csv("city-gps.txt", sep=" ", names=['city', 'latitude', 'longitude'])
road_segments = pd.read_csv("road-segments.txt", sep=" ", names=['cityone', 'citytwo', 'length', 'speed', 'highway'])
avg_Speed = road_segments.speed.mean()
road_segments.replace(np.nan, avg_Speed, inplace=True)
road_segments.replace(0, avg_Speed, inplace=True)
road_segments['time'] = road_segments['length'] / road_segments['speed']
road_segments_lis = road_segments.values.tolist()
city_gps_lis = city_gps.values.tolist()

# Dictionary having  cities as keys and Distance and time taken as values
length_Timedict = {}
for lis in road_segments_lis:
    length_city = lis[2]
    time_city = lis[5]
    cities = tuple(lis[:2])
    length_Timedict[cities] = length_city, time_city
    temp = lis[0]
    lis[0] = lis[1]
    lis[1] = temp
    length_city = lis[2]
    time_city = lis[5]
    cities = tuple(lis[:2])
    length_Timedict[cities] = length_city, time_city

# Dictioanry having City as key and latitude and longitude as values
latitude_Longitudedict = {}
for lis in city_gps_lis:
    city_Name = lis[0]
    lat = lis[1]
    long = lis[2]
    latitude_Longitudedict[city_Name] = lat, long

solution = solve(road_segments)

if solution == goal_state:
    print('yes', 0, 0, initial_state, initial_state)
else:
    if (initial_state not in road_segments.iloc[:, 0].values and initial_state not in road_segments.iloc[:,
                                                                                      1].values) or (
            goal_state not in road_segments.iloc[:, 0].values and goal_state not in road_segments.iloc[:, 1].values):
        print("Check City Names!!!")
    else:
        if routing_algo == 'ids':
            idsDepth = 0
            solution = False
            while solution == False:
                solution = solve(road_segments, idsDepth)
                idsDepth = idsDepth + 1
        else:
            solution = solve(road_segments)

        solution_Copy = solution
        parent_City_lis = []
        while solution.parentCityState is not None:
            x = solution.parentCityState.cityName
            parent_City_lis.append(x)
            solution = solution.parentCityState

        parent_City_lis = list(reversed(parent_City_lis))
        parent_City_lis.append(goal_state)

        optimal_AlgoList = ['astar', 'uniform']
        isOptimal = 'no'
        if routing_algo in optimal_AlgoList or (
                (routing_algo == 'bfs' or routing_algo == 'ids') and cost_function == 'segments'):
            isOptimal = 'yes'

        print(isOptimal, solution_Copy.distance, solution_Copy.time, ' | '.join(parent_City_lis))
