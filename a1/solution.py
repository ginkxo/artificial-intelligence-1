#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Snowman Puzzle domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes and problems
from test_problems import PROBLEMS #20 test problems

# extra import 
import math

#snowball HEURISTICS
def heur_simple(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_zero(state):
  return 0

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible snowball puzzle heuristic: manhattan distance'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between the snowballs and the destination for the Snowman is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    # between each snowball, or stack of snowballs, and destination


    manhattan = 0;
    dest_x = state.destination[0]
    dest_y = state.destination[1]

    for coordinates, size in state.snowballs.items():

      manhattan_val = abs(coordinates[0] - dest_x) + abs(coordinates[1] - dest_y)
      if size < 3:
        manhattan += manhattan_val
      else:
        manhattan += manhattan_val * 2

    return manhattan 


def heur_alternate(state): 
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    '''
    Heuristic idea:

    manhattan distance + sum of total weights of snowballs not on goal
    expands upon manhattan distance, but 
      for each snowball_tuple among the snowballs, as long as the snowball's 
      positions are not equal to the goal (aka they are on the goal),
      then we add the weight to the manhattan distance value 

    should improve because it gives an ordering?
    manhattan distance alone will just focus on getting the closest ones
    but with this, we will also try and minimize heuristic values 
    through getting big -> medium -> small 

    '''

    # idea 1: robot to destination
    # idea 2: robot to snowballs 

    # going to add the corner checking

    manhattan = 0
    blocked = 0
    robo_dist = 0

    dest_x = state.destination[0]
    dest_y = state.destination[1]

    robot_x_diff = state.robot[0] - dest_x
    robot_y_diff = state.robot[1] - dest_y

    robo_dist = abs(robot_x_diff) + abs(robot_y_diff)

    wh = state.height - 1
    ww = state.width - 1

    #robo_sb_dist = []
    robo_sb_dist = 2**(16)

    wall_corners = {(0,0), (0, wh), (ww, 0), (ww, wh)}

    for coordinates, size in state.snowballs.items():

      sb_x = coordinates[0]
      sb_y = coordinates[1]

      left = (sb_x - 1, sb_y)
      right = (sb_x + 1, sb_y)
      up = (sb_x, sb_y + 1)
      down = (sb_x, sb_y - 1)

      manhattan_val_x = abs(sb_x - dest_x) 
      manhattan_val_y = abs(sb_y - dest_y)

      new_robo_sb_dist = abs(state.robot[0] - sb_x) + abs(state.robot[1] - sb_y)
      #robo_sb_dist.append(new_robo_sb_dist)
      if new_robo_sb_dist <= robo_sb_dist:
        robo_sb_dist = new_robo_sb_dist

      if size < 3:
        manhattan += manhattan_val_x + manhattan_val_y

        if (state.destination != coordinates):

          if (sb_x == 0 and dest_x != 0):
            blocked = 2**(20)
            return blocked

          elif (sb_x == ww and dest_x != ww):
            blocked = 2**(20)
            return blocked

          elif (sb_y == 0 and dest_y != 0):
            blocked = 2**(20)
            return blocked

          elif (sb_y == wh and dest_y != ww):
            blocked = 2**(20)
            return blocked
                        
          elif (coordinates in wall_corners):

            blocked = 2**(20)
            return blocked

          else:

            if (left in state.obstacles or right in state.obstacles):

              if (up in state.obstacles or down in state.obstacles):

                blocked = 2**(20)
                return blocked

              elif (sb_y == 0 or sb_y == wh):

                blocked = 2**(20)
                return blocked

            elif (up in state.obstacles or down in state.obstacles):

              if (sb_x == 0 or sb_x == ww):

                blocked = 2**(20)
                return blocked 
      else:
        manhattan += (manhattan_val_x + manhattan_val_y) * 2

      cross = abs((manhattan_val_x * robot_y_diff) - (manhattan_val_y * robot_x_diff))
      manhattan += cross * 0.001


    return manhattan + blocked + (robo_dist*0.5) + robo_sb_dist

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SnowballState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    fval = sN.gval + weight * sN.hval 
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return fval 

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 

    
    max_timebound = os.times()[0] + timebound 
    better_goal = True 
    searcher = SearchEngine('best_first', 'full')
    searcher.init_search(initial_state, snowman_goal_state, heur_fn)
    goal = searcher.search(max_timebound - os.times()[0])

    while goal and better_goal:
      goal_g = (goal.gval, float("inf"), float("inf"))
      better_goal = searcher.search(max_timebound - os.times()[0], goal_g)
      if (better_goal):
        goal = better_goal 

    return goal


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''

    max_timebound = os.times()[0] + timebound
    better_goal = True
    searcher = SearchEngine('custom', 'full') 
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    searcher.init_search(initial_state, snowman_goal_state, heur_fn, wrapped_fval_function)
    goal = searcher.search(max_timebound - os.times()[0])

    while goal and better_goal:
      goal_f = (float("inf"), float("inf"), goal.gval)
      better_goal = searcher.search(max_timebound - os.times()[0], goal_f)
      if (better_goal):
        goal = better_goal

    return goal 

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 20 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=snowman_goal_state, heur_fn=heur_simple)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10 
    final = anytime_weighted_astar(s0, heur_fn=heur_simple, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 
