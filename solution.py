#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    min_distances = []
    for box in state.boxes:
      distance = []
      for spot in state.storage:
        del_x = abs(box[0] - spot[0])
        del_y = abs(box[1] - spot[1])
        distance.append(del_x + del_y)
      min_distances.append(min(distance))

    return sum(min_distances)


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    min_dist_box_to_robot = []
    for box in state.boxes:
      dist_box_to_robot = []
      for robot in state.robots:
        del_x = abs(robot[0] - box[0])
        del_y = abs(robot[1] - box[1])
        dist_box_to_robot.append(del_x + del_y - 1)
      min_dist_box_to_robot.append(min(dist_box_to_robot))
    
    min_dist_box_to_spot = []

    for box in state.boxes:
      dist_box_to_spot = []
      #if box[0] != 0 and box[0] != (state.width - 1) and box[1] != 0 and box[0] != (state.height - 1):   
      for spot in state.storage:
        #found = True
        del_x = abs(box[0] - spot[0])
        del_y = abs(box[1] - spot[1])
        dist_box_to_spot.append(del_x + del_y)
      '''else:
        found = False
        if box[0] == 0:
          for spot in state.storage:
            if spot[0] == 0:
              found = True
              del_x = abs(box[1] - spot[1])
              dist_box_to_spot.append(del_x)

        elif box[0] == state.width - 1:
          for spot in state.storage:
            if spot[0] == state.width - 1:
              found = True
              del_x = abs(box[1] - spot[1])
              dist_box_to_spot.append(del_x)

        elif box[1] == 0:
          for spot in state.storage:
            if spot[0] == 0:
              found = True
              del_y = abs(box[0] - spot[0])
              dist_box_to_spot.append(del_y)

        elif box[1] == state.height - 1:
          for spot in state.storage:
            if spot[0] == state.height - 1:
              found = True
              del_y = abs(box[0] - spot[0])
              dist_box_to_spot.append(del_y)'''

      found1 = False
      found2 = False
      found3 = False
      for spot in state.storage:
        if spot[0] == 0 and spot[1] == 0:
          found1 = True
        elif spot[0] == 0 and spot[1] == 1:
          found2 = True
        elif spot[0] == 1 and spot[1] == 0:
          found3 = True
        if found1 and found2 and found3:
          min_dist_box_to_spot.append(max(dist_box_to_spot))
      if not found1 or not found2 or not found3:
        min_dist_box_to_spot.append(min(dist_box_to_spot))
      #else:
        #return float("inf")

    min_distances = [sum(i) for i in zip(min_dist_box_to_robot, min_dist_box_to_spot)]  
    return sum(min_distances)

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval



def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  initial_time = os.times()[0]
  se = SearchEngine('custom')
 # sN = sNode(initial_state, trivial_heuristic, fval_function)
  wrapped_fval_function = (lambda sN :fval_function(sN,weight))
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
  goal_1 = se.search(timebound, None)
  if goal_1 != False:
    while(1 == 1):
      goal_2 = se.search(timebound - (os.times()[0] - initial_time), [100000000, 10000000000, weight])
      if goal_2 != False:
        goal_1 = goal_2
      else:
        return goal_1

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  initial_time = os.times()[0]
  se = SearchEngine('best_first')
  se.init_search(initial_state, sokoban_goal_state, heur_fn)
  goal_1 = se.search(timebound, None)
  if goal_1 != False:
    while(1 == 1):
      goal_2 = se.search(timebound - (os.times()[0] - initial_time), [goal_1.gval, 10000000000, 1000000000])
      if goal_2 != False:
        goal_1 = goal_2
      else:
        return goal_1
