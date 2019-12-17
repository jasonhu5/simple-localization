#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
import yaml
import re

class SimpleMap():
    ''' INITIALIZE '''

    def __init__(self, map_file):
        # Read yaml file
        with open(map_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                # Add vertices for all the floor tags.
                map_raw = complete_dict.get("tiles")
                tile_size = complete_dict.get("tile_size")
                # TODO: what happens if any is None
            except yaml.YAMLError as exc:
                print(exc)

        # Map specific parameters
        # https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html
        self.tile_size = tile_size
        self.margin = 0.08

        # Tile specific orientation
        # - locations of center(s) of curve
        self.curve_left_c = {
            "N":[(0, 0)],
            "E":[(0, 1)],
            "S":[(1, 1)],
            "W":[(1, 0)],
        }
        self.curve_right_c = {
            "N":[(1, 0)],
            "E":[(0, 0)],
            "S":[(0, 1)],
            "W":[(1, 1)],
        }
        self.threeway_left_c = {
            "N":[(0, 0), (0, 1)],
            "E":[(0, 1), (1, 1)],
            "S":[(1, 1), (1, 0)],
            "W":[(1, 0), (0, 0)],
        }
        self.threeway_right_c = {
            "N":[(1, 0), (1, 1)],
            "E":[(0, 0), (1, 0)],
            "S":[(0, 1), (0, 0)],
            "W":[(1, 1), (0, 1)],
        }
        self.fourway_c = [(0, 0), (0, 1), (1, 0), (1, 1)]
        # - possible boarder positions
        self.boarders = ["N", "E", "S", "W"]


        # rotate map to suit localization output
        map_raw_corrected = [r[::-1] for r in zip(*map_raw)]

        # Raw string version from yaml file
        self.map_raw = map_raw_corrected

        # Convert to binary on road/off road
        # 1 = street; 0 = no street
        self.map_bin = np.array(
            [[int(v != "asphalt") for v in r] for r in self.map_raw]
        )

        # Convert to a state: lane/intersection/out_of_lane
        self.map_state = np.array(
            [["out_of_lane" if "asphalt" in v
            else "intersection" if "way" in v
            else "lane" for v in r] for r in self.map_raw]
        )

        # Convert to semantic with orientation
        self.map_sem = self.raw2sem(self.map_raw)


    def raw2sem(self, map_raw):
        # Return a semantic map with tile type and orientations
        # orientation given through coordinates of curve center
        map_dict = dict()
        numrows = len(map_raw)
        numcols = len(map_raw[0])
        for row in range(numrows):
            for col in range(numcols):
                tile = re.split("[/_]", map_raw[row][col])
                if not self.try_orientation(tile[-1]): return None
                if "asphalt" in tile[0]:
                    map_dict[(row, col)] = Tile("asphalt", [], self.boarders)
                elif "4way" in tile[0]:
                    centers = self.fourway_c
                    map_dict[(row, col)] = Tile("4way", centers, [])
                elif "straight" in tile[0]:
                    if "S" in tile[-1] or "N" in tile[-1]:
                        boarders = ["N", "S"]
                    else:
                        boarders = ["E", "W"]
                    map_dict[(row, col)] = Tile("straight", [], boarders)
                elif "curve" in tile[0]:
                    if "left" in tile[1]:
                        centers = self.curve_left_c[tile[-1]]
                    elif "right" in tile:
                        centers = self.curve_right_c[tile[-1]]
                    map_dict[(row, col)] = Tile("curve", centers, [])
                elif "3way" in tile[0]:
                    if "left" in tile[1]:
                        centers = self.threeway_left_c[tile[-1]]
                        o = self.boarders.index(tile[-1])+2
                        boarders = self.boarders[o%len(self.boarders)]
                    elif "right" in tile:
                        centers = self.threeway_right_c[tile[-1]]
                        o = self.boarders.index(tile[-1])-1
                        boarders = self.boarders[o%len(self.boarders)]
                    map_dict[(row, col)] = Tile("3way", centers, ['{}'.format(boarders)])
                else:
                    print("Unknown Tile"); return None
        return map_dict


    ''' POSITIONING FUNCTIONS'''
    def position_on_map(self, position, subtile=False):
        # Returns higher-level information on the position_on_map
        # depending on the map (bin, sem, etc.)
        if subtile:
            map = self.map_sem
        else:
            map = self.map_bin
        # Check if valid position
        if self.get_tile(position) is not None:
            tile_x, tile_y = self.get_tile(position)
        else:
            return None
        if subtile:
            # So far only curve and asphalt criterion
            tile = self.map_sem[(tile_x, tile_y)]
            on_lane = 1
            if type(map) is not dict:
                print("Passed wrong map as arg, need semantic"); return
            if tile.type == "asphalt":
                on_lane = 0
            elif tile.type == "curve":
                # Calculate disstance from center of curve
                x = abs(tile.centers[0][0]*self.tile_size - (position[0] % self.tile_size))
                y = abs(tile.centers[0][1]*self.tile_size - (position[1] % self.tile_size))
                if math.sqrt(x**2+y**2) > self.tile_size:
                    on_lane = 0
                # print("Distances from center: x = {}, and y = {}".format(x,y))
            return on_lane
        else:
            if type(map) is not np.ndarray:
                print("Passed wrong map as arg, need binary"); return
            return map[tile_x, tile_y]

    def pos_to_ideal_heading(self, position):
        # Returns the correct angle a duckiebot should have for a
        # certain position passed as argument
        # Returns None for positions on 4/way, 3/way or asphalt tile
        # Check if valid position
        if self.get_tile(position) is not None:
            tile_x, tile_y = self.get_tile(position)
        else:
            return None
        tile = self.map_sem[(tile_x, tile_y)]
        heading = None
        if tile.type == "straight":
            if tile.boarders == ["N", "S"]:
                lane = round((position[0] % self.tile_size)/self.tile_size)
                heading = -90+lane*180
            else:
                lane = round((position[1] % self.tile_size)/self.tile_size)
                heading = lane*180
        elif tile.type == "curve":
            if(self.position_on_map(position, subtile=True)):
                # get center of 1/4 circle (= curve)
                center_x = tile.centers[0][0]
                center_y = tile.centers[0][1]
                # calculate distance from center
                x = abs(center_x*self.tile_size - (position[0] % self.tile_size))
                y = abs(center_y*self.tile_size - (position[1] % self.tile_size))
                # check if on inner or outer lane
                lane = round(math.sqrt(x**2+y**2)/self.tile_size)
                print("outer lane") if lane else "inner lane"
                # convert center from binary to {-1;1} for atan2 calculations
                curve_sign_x = 2*(center_x-0.5)
                curve_sign_y = 2*(center_y-0.5)
                # calculate angle in relative coordinate system
                heading = -math.atan2(y*curve_sign_y,x*curve_sign_x)
                # convert angle to global coordinate system and account for lane
                # turn by 90 deg and reverse for other lane (+180 deg)
                heading = self.normalize_angle((0.5+lane)*math.pi - heading)
        return heading

    def pos_to_ideal_position(self, position):
        if self.get_tile(position) is not None:
            tile_x, tile_y = self.get_tile(position)
        else:
            return None
        tile = self.map_sem[(tile_x, tile_y)]
        coordinates = None
        if tile.type == "straight":
            if tile.boarders == ["N", "S"]:
                lane = round((position[0] % self.tile_size)/self.tile_size)
                coordinates = ((tile_x+0.25+lane/2)*self.tile_size, position[1])
            else:
                lane = round((position[1] % self.tile_size)/self.tile_size)
                coordinates = (position[0], (tile_y+0.25+lane/2)*self.tile_size)
        elif tile.type == "curve":
                center_x = tile.centers[0][0]
                center_y = tile.centers[0][1]
                # calculate distance from center
                x = abs(center_x*self.tile_size - (position[0] % self.tile_size))
                y = abs(center_y*self.tile_size - (position[1] % self.tile_size))
                # check if on inner or outer lane
                lane = round(math.sqrt(x**2+y**2)/self.tile_size)
                # calculate position using intercept theorem
                base_x = (tile_x + center_x)*self.tile_size
                base_y = (tile_y + center_y)*self.tile_size
                distance = math.sqrt(x**2+y**2)
                coord_x = base_x + x * ((0.25+lane/2)*self.tile_size)/distance
                coord_y = base_y + y * ((0.25+lane/2)*self.tile_size)/distance
                coordinates = (coord_x, coord_y)
        return coordinates


    ''' ERROR HANDLING '''
    def try_orientation(self, o):
        orientations = ['N', 'E', 'S', 'W', 'asphalt', '4way']
        if any(x in o for x in orientations):
            return True
        else:
            print("Unknown orientation"); return False

    def try_tile_idx(self, index):
        i,j = index
        if i*j >= 0 and i < self.map_bin.shape[0] and j < self.map_bin.shape[1]:
            return True
        else:
            # print("Position not on map");
            return False



    ''' PRINT FUNCTIONS FOR DEBUGGING '''
    def display_raw_debug(self):
        # >>> print(u'\u2551')
        # ║
        # >>> print(u'\u2550')
        # ═
        # >>> print(u'\u2554')
        # ╔
        # >>> print(u'\u2557')
        # ╗
        # >>> print(u'\u255A')
        # ╚
        # >>> print(u'\u255D')
        # ╝
        # >>> print(u'\u2560')
        # ╠
        # >>> print(u'\u2563')
        # ╣
        # >>> print(u'\u2566')
        # ╦
        # >>> print(u'\u2569')
        # ╩
        # >>> print(u'\u256c')
        # ╬

        self.display_matrix(
            [['o' if 'asphalt' in v else
            u'\u2550' if ('straight/S' in v or 'straight/N' in v) else
            u'\u2551' if ('straight/W' in v or 'straight/E' in v) else
            u'\u2557' if 'curve_left/W' in v else
            u'\u255D' if 'curve_left/N' in v else
            u'\u255A' if 'curve_right/S' in v else
            u'\u2554' if 'curve_right/W' in v else
            u'\u2554' if 'curve_left/S' in v else
            u'\u2563' if '3way_left/W' in v else
            u'\u2569' if '3way_left/N' in v else
            u'\u256C' for v in r] for r in self.map_raw],
            "Raw map debug",
            ' '
        )

    def display_sem_debug(self):
        # Display semantic map for debugging
        numrows = len(self.map_raw)
        numcols = len(self.map_raw[0])
        for row in range(numrows):
            print(' ')
            for col in range(numcols):
                print(self.map_sem[(row,col)])

    ''' HELPER FUNCTIONS '''

    def get_tile(self, position):
        # returns the indices of the tile from current position
        tile_x = int(math.floor(position[0]/self.tile_size))
        tile_y = int(math.floor(position[1]/self.tile_size))
        if not self.try_tile_idx((tile_x, tile_y)): return None
        return tile_x, tile_y

    def normalize_angle(self, angle):
        # takes an angle in radians and converts it to degrees in (-180;180]
        newAngle = 180/math.pi*angle
        while (newAngle <= -180): newAngle += 360
        while (newAngle > 180): newAngle -= 360
        return round(newAngle)

    def display_matrix(self, mat, name=None, sep_word="", sep_line="\n"):
        print("\n<%s>" % name if name else "\n")
        print(sep_line.join([sep_word.join(r) for r in mat]))
        print("\n")


class Tile():

    def __init__(self, type, centers=[], boarders=[]):
        # Only used to store min parameters to determine type and orientation
        self.type = type
        self.centers = centers
        self.boarders = boarders

    def __str__(self):
        tile = "Type = {}, Centers = {}, Boarders =  {}"
        return tile.format(self.type, self.centers, self.boarders)
