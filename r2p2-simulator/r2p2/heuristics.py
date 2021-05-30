""" This module implements several heuristics.

They assume the Node class provided with this simulator is being used.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Mario Cobos Maestre"
__authors__ = ["Mario Cobos Maestre"]
__contact__ = "mario.cobos@edu.uah.es"
__copyright__ = "Copyright 2019, UAH"
__credits__ = ["Mario Cobos Maestre"]
__date__ = "2019/03/29"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"


import path_planning as pp
import math

def manhattan(node_point,goal_point):
    """
        Function that performs Manhattan heuristic.
    """
    x1,y1 = node_point.grid_point
    x2,y2 = goal_point.grid_point
    return abs(x1 - x2) + abs(y1 - y2)

pp.register_heuristic('manhattan', manhattan)

def naive(node_point,goal_point):
    """
        Function that performs a naive heuristic.
    """
    return 1

pp.register_heuristic('naive', naive)

def euclidean(node_point,goal_point):
    """
        Function that performs euclidean heuristic.
    """
    x1,y1 = node_point.grid_point
    x2,y2 = goal_point.grid_point
    return math.sqrt((x2 - x1)^2 + (y2 - y1)^2)

pp.register_heuristic('euclidean', euclidean)

def octile(node_point,goal_point):
    """
        Function that performs octile heuristic.
    """
    x1,y1 = node_point.grid_point
    x2,y2 = goal_point.grid_point
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return math.sqrt(1^2 + 1^2) * min(dx, dy) + abs(dx - dy)

pp.register_heuristic('octile', octile)