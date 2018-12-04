#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import *
from tf import *
from collections import defaultdict

import numpy


class Mover():
    def __init__(self, goal_x, goal_y):

        # subscribe to transforms
        self.tf = TransformListener()

        # some variables we'll use later
        # self.map_x
        # self.map_y
        # self.grid_res
        # self.grid_w
        # self.grid_h

        # the goal is 2-space in map frame
        self.goal_x = goal_x
        self.goal_y = goal_y

        # subscribe to /map topic to get the occupancy grid 
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # subscribe to /odom topic to get the position of the robot with respect to the odom origin
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size =10)

        # subscribe to navigation_end topic that sends out 'end' when navigation has ended
        rospy.Subscriber('navigation_end', String, self.navigation_end_cb)

        # publisher for the sequence of navigation coords
        self. navigation_coord_sequence_publisher = rospy.Publisher('navigation_coord_sequence', String, queue_size= 10) ###

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def map_callback(self, data):
        rospy.loginfo("Got map!")

        map_header = data.header
        map_meta_data = data.info
        self.grid_origin_x = map_meta_data.origin.position.x
        self.grid_origin_y = map_meta_data.origin.position.y
        self.grid_data = data.data

        self.grid_res = map_meta_data.resolution
        self.grid_w = map_meta_data.width
        self.grid_h = map_meta_data.height

        rospy.loginfo('Got new map with res %s', self.grid_res)

    def odom_callback(self, data):
        pose = data.pose.pose #  the x,y,z pose and quaternion orientation
        # rospy.loginfo('Got new odom %s', pose)

        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        # transform the pose in the odom fram to the map frame
        self.odom_to_map_transformer(self.odom_x, self.odom_y)
        self.map_x, self.map_y = self.odom_to_map_transformer(self.odom_x, self.odom_y)

        # get the occupancy grid pixels the correlate to the map position
        pixel_x, pixel_y = self.map_coord_to_pixel(self.map_x, self.map_y)

        # get the value of the occupancy grid in that pixel
        pixel_value = self.grid_pixel_value(pixel_x, pixel_y)
        # self.show_surrounding_pixels()


        # rospy.loginfo('transform to map frame %s', self.odom_to_map_transformer(self.odom_x, self.odom_y))
        # rospy.loginfo('transform to map again %s', self.pixel_to_map_coord(pixel_x, pixel_y))


        self.current_pixel = (pixel_x,pixel_y)

    def navigation_end_cb(self, data):
        '''
        When this callback is fired, it means that the robot has reached it's goal
        We would want to send another path to a goal - calculated using Dijkstra
        '''
        print('GOT THIS ', data, 'FROM NAVIGATION END TOPIC')
        print('caculate dijkstra')

        goal_pixel = self.map_coord_to_pixel(self.goal_x,self.goal_y) ### the goal########################################
        goal_pixel_x = goal_pixel[0]
        goal_pixel_y = goal_pixel[1]
        path = tuple(self.dijkstra_to_goal(self.weighted_graph(), self.current_pixel, goal_pixel))

        path_string = ''

        for coord_index in range(len(path)-1):
            coord = path[coord_index]
            next_coord = path[coord_index +1]

            x_coord = coord[0]
            y_coord = coord[1]
            print('this', x_coord, y_coord)

            next_x_coord = next_coord[0]
            next_y_coord = next_coord[1]
            print('next', next_x_coord, next_y_coord)

            # calculate yaw by the slope to the next coord

            try:
                yaw_coord = numpy.arctan((next_y_coord - y_coord)/(next_x_coord - x_coord))*180/numpy.pi
            except ZeroDivisionError:
                yaw_coord = 90


            path_string += str(x_coord) + ',' + str(y_coord) + ',' + str(yaw_coord) + ','
            print(coord)
        
        # add the last coord to the string, with yaw of 90

        path_string += str(next_x_coord) + ',' + str(next_y_coord) + ',' + str(90)



        # take out last comma
        # path_string = path_string[:-1]

        print('publishing a string of the coords, all with 90 degrees yaw')
        print(path_string)

        self.navigation_coord_sequence_publisher.publish(path_string)


    def odom_to_map_transformer(self, init_x, init_y):
        '''
        return a new tuple, that is now correct with reference to the map frame (x,y)
        '''
        (trans,rot) = self.tf.lookupTransform('/map','/odom',rospy.Time(0))

        delta_x = trans[0]
        delta_y = trans[1]
        return (init_x + delta_x, init_y + delta_y)        

    def map_coord_to_pixel(self, x,y):
        '''
        returns the row and col that correspond to the map-frame coordinates in the occupancy map
        '''
        # localize the robot on the occupancy grid
        pixel_x = round((x - self.grid_origin_x)/self.grid_res)
        pixel_y = round((y - self.grid_origin_y)/self.grid_res)
        return (pixel_x, pixel_y)

    def pixel_to_map_coord(self, pixel_x, pixel_y):
        '''
        converts occupancy grid pixel value to coord in map frame
        '''
        map_x = pixel_x*self.grid_res + self.grid_origin_x
        map_y = pixel_y*self.grid_res + self.grid_origin_y
        return (map_x, map_y)

    def grid_pixel_value(self, pixel_x, pixel_y):
        '''
        return the occupancy value [0,100] of the pixel of interest in the occupancy grid
        '''
        
        grid_index = int(pixel_x + pixel_y * self.grid_w)
        # rospy.loginfo('we are at x=%s y=%s in the grid, with value %s', pixel_x, pixel_y, self.grid_data[grid_index])

        return self.grid_data[grid_index]

    def show_surrounding_pixels(self):
        surrounding_list = []
        for i in range(-10, 10):
            x_delta = i/10.0
            x, y = self.map_coord_to_pixel(self.map_x + x_delta, self.map_y)
            surrounding_list.append(self.grid_pixel_value(x,y))
        print(surrounding_list)


    def get_possible_coords(self, x, y, grid = None):

        if grid == None:
            grid = self.grid_data

        # the allowed moves are up down left right
        # return a list of tuples that only include coordinates of cells that are empty
        tries = [(x, y+1), (x, y-1), (x+1, y), (x-1, y), (x-1,y+1), (x+1,y-1), (x+1, y+1), (x-1, y-1)]

        # add a test for outer obstacles, so won't be too close to any obstacle (account for robot width)
        # maybe d = 10 pixels to the side
        finals = []
        for coord in tries:
            if coord[0] < self.grid_w and coord[0] >= 0 and coord[1] < self.grid_h and coord[1] >= 0:
                if self.get_item(coord[0], coord[1], grid) in [0,'G']:
                    finals.append(coord)
        return finals 

    def weighted_graph(self):
        weighted_graph = {}
        for x in range(self.grid_w):
            for y in range (self.grid_h):
                coord = (x,y)
                # get surrounding coords
                neighbors = self.get_possible_coords(x,y,self.grid_data)
                # add a dict {node:{neighbor1:dist1, neighbor2: dist2}} to weighted graph. In the simple grid, all distances will be 1
                neighbors_dict = {}
                for neighbor in neighbors:
                    neighbors_dict[neighbor] = 1
                # add the node with its neighbors to the weighted graph
                weighted_graph[coord] = neighbors_dict
        return weighted_graph

    def x_y_to_index(self, x, y):
        return x + y*self.grid_w

    def get_item(self, x, y, grid = None):
        if grid == None:
            grid = self.grid_data
        return grid[self.x_y_to_index(x,y)]
    
    def dijkstra_to_goal(self, weighted_graph, start, end):
        '''
        generates a list of (x,y) coords in the grid frame from current position to specified goal
        which is also (x,y) in grid frame
        '''
        """
        Calculate the shortest path for a directed weighted graph.

        Node can be virtually any hashable datatype.

        :param start: starting node
        :param end: ending node
        :param weighted_graph: {"node1": {"node2": "weight", ...}, ...}
        :return: ["START", ... nodes between ..., "END"] or None, if there is no
                path
        """

        # We always need to visit the start
        nodes_to_visit = {start}
        visited_nodes = set()
        distance_from_start = defaultdict(lambda: float("inf"))
        # Distance from start to start is 0
        distance_from_start[start] = 0
        tentative_parents = {}

        while nodes_to_visit:
            # The next node should be the one with the smallest weight
            current = min(
                [(distance_from_start[node], node) for node in nodes_to_visit])[1]

            # The end was reached
            if current == end:
                break

            nodes_to_visit.discard(current)
            visited_nodes.add(current)

            # print('weighted_graph[current]', weighted_graph[current])

            for neighbour, distance in weighted_graph[current].items():
                if neighbour in visited_nodes:
                    continue
                neighbour_distance = distance_from_start[current] + distance
                if neighbour_distance < distance_from_start[neighbour]:
                    distance_from_start[neighbour] = neighbour_distance
                    tentative_parents[neighbour] = current
                    nodes_to_visit.add(neighbour)

        return self._deconstruct_path(tentative_parents, end)


    def _deconstruct_path(self, tentative_parents, end):
        if end not in tentative_parents:
            return None
        cursor = end
        path = []
        while cursor:
            path.append(cursor)
            cursor = tentative_parents.get(cursor)

        # convert to map frame

        pixel_list = list(reversed(path))
        map_coord_list= []
        for pixel_x, pixel_y in pixel_list:
            map_coord_list.append((self.pixel_to_map_coord(pixel_x, pixel_y)))
        return map_coord_list
                





        
    
def main():
    # initialize node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mover', anonymous=True)

    # create new instance of the class
    mover = Mover(1.8,1.5)


    


if __name__ == '__main__':
    main()

