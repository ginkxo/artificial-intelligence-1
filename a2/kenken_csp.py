#Look for #IMPLEMENT tags in this file.

'''
Construct and return Kenken CSP model.
'''

from cspbase import *
import itertools

def kenken_csp_model(kenken_grid):
    '''Returns a CSP object representing a Kenken CSP problem along 
       with an array of variables for the problem. That is return

       kenken_csp, variable_array

       where kenken_csp is a csp representing the kenken model
       and variable_array is a list of lists

       [ [  ]
         [  ]
         .
         .
         .
         [  ] ]

       such that variable_array[i][j] is the Variable (object) that
       you built to represent the value to be placed in cell i,j of
       the board (indexed from (0,0) to (N-1,N-1))

       
       The input grid is specified as a list of lists. The first list
	   has a single element which is the size N; it represents the
	   dimension of the square board.
	   
	   Every other list represents a constraint a cage imposes by 
	   having the indexes of the cells in the cage (each cell being an 
	   integer out of 11,...,NN), followed by the target number and the
	   operator (the operator is also encoded as an integer with 0 being
	   '+', 1 being '-', 2 being '/' and 3 being '*'). If a list has two
	   elements, the first element represents a cell, and the second 
	   element is the value imposed to that cell. With this representation,
	   the input will look something like this:
	   
	   [[N],[cell_ij,...,cell_i'j',target_num,operator],...]
	   
       This routine returns a model which consists of a variable for
       each cell of the board, with domain equal to {1-N}.
       
       This model will also contain BINARY CONSTRAINTS OF NOT-EQUAL between
       all relevant variables (e.g., all pairs of variables in the
       same row, etc.) and an n-ary constraint for each cage in the grid.
    '''

    ##IMPLEMENT

    kenken_csp = CSP("kenken")



    N = kenken_grid[0][0]
    constraint_number = len(kenken_grid) - 1
    domain = [i for i in range(1, N+1)] 

    var_board = [[None for i in range(N)] for i in range(N)]
    var_list = []

    for cstr_i in range(1, constraint_number + 1):

      constraint = kenken_grid[cstr_i]

      if len(constraint) == 2:

        var = Variable(str(constraint[0]), domain)
        kenken_csp.add_var(var)
        var_list.append(var)
        str_d_1 = int(str(constraint[0])[0]) - 1
        str_d_2 = int(str(constraint[0])[1]) - 1
        var_board[str_d_1][str_d_2] = var 

        cnstr = Constraint(str(constraint[0]), [var])
        cnstr.add_satisfying_tuples([[constraint[1]]])
        kenken_csp.add_constraint(cnstr)

      elif len(constraint) > 2:

        # the larger constraints

        cst_scope = []
        lenc = len(constraint)

        for varval in range(0, lenc-2):

          var = Variable(str(constraint[varval]), domain)
          kenken_csp.add_var(var)
          var_list.append(var)
          cst_scope.append(var)
          str_d_1 = int(str(constraint[varval])[0]) - 1
          str_d_2 = int(str(constraint[varval])[1]) - 1
          var_board[str_d_1][str_d_2] = var 

        cst = Constraint(str((constraint[lenc-2], constraint[lenc-1])), cst_scope)
        satisfying_tuples = satisfy(lenc-2, constraint[lenc-2], constraint[lenc-1], N)
        cst.add_satisfying_tuples(satisfying_tuples) # based on function
        kenken_csp.add_constraint(cst) 

    # binary not equal tuples

    neq_array = bin_not_equal(N)

    for row in range(N):
      for col in range(N):
        for horiz in range(col+1, N):
          cst = Constraint(var_board[row][col].name + " " + var_board[row][horiz].name, [var_board[row][col], var_board[row][horiz]])
          cst.add_satisfying_tuples(neq_array)
          kenken_csp.add_constraint(cst)
        for vert in range(row+1, N):
          cst = Constraint(var_board[row][col].name + " " + var_board[vert][col].name, [var_board[row][col], var_board[vert][col]])
          cst.add_satisfying_tuples(neq_array)
          kenken_csp.add_constraint(cst)


    # comment out the below tbh

    '''
    for i in range(N):

      horiz_vars = [x for x in var_list if x.name[0] == str(i+1)]
      vert_vars = [y for y in var_list if y.name[1] == str(i+1)]
      horizontal = Constraint(str("H" + str(i+1)), horiz_vars)
      vertical = Constraint(str("V" + str(i+1)), vert_vars)

      horizontal.add_satisfying_tuples(sat_alldifs(N))
      kenken_csp.add_constraint(horizontal)
      vertical.add_satisfying_tuples(sat_alldifs(N))
      kenken_csp.add_constraint(vertical)

    '''
    # new constraint - for horizontal rows
      # iterate through the variable list, match based on first digit
    # new constraint - for vertical columns
      # iterate through the variable list, match based on second digit

    # need an alldiff function

    return kenken_csp, var_board 


def check_diff(var_list):

  good_tuples = []
  vardoms = []
  for variable in var_list:
    vardoms.append(variable.domain())

  for possible_tuple in itertools.product(*vardoms):

    if alldiff(possible_tuple):
      good_tuples.append(possible_tuple)

  return good_tuples


def alldiff(tupleval):

  varlistlen = len(tupleval)
  bitvector = [0 for i in range(varlistlen)]

  for digit in tupleval:

      if bitvector[tupleval] == 1:

        return False

      else:

        bitvector[tupleval] = 1

  return True 

def bin_not_equal(N):

  x = [i for i in range(1, N+1)]
  y = []
  for t in itertools.permutations(x, 2):
    y.append(t)
  return y

def sat_alldifs(N):

  sat_tuples = []
  vardoms = [i for i in range(1, N+1)]
  for t in itertools.permutations(vardoms):
    sat_tuples.append(t)

  return sat_tuples

def satisfy(num_vars, target, operation, N):

  t_satisfied = []
  t_potential = []
  t_args = []

  t_domain = [i for i in range(1, N+1)]

  for z in range(num_vars):

    t_args.append(t_domain)

  for t in itertools.product(*t_args):

    t_potential.append(t)

  if operation == 0:

    for potential_t in t_potential:
      if check_sum(potential_t, target):
        t_satisfied.append(potential_t)

    return t_satisfied

  elif operation == 1:

    for potential_t in t_potential:
      if check_sub(potential_t, target):
        t_satisfied.append(potential_t)

    return t_satisfied

  elif operation == 2:

    for potential_t in t_potential:
      if check_div(potential_t, target):
        t_satisfied.append(potential_t)

    return t_satisfied

  elif operation == 3:

    for potential_t in t_potential:
      if check_mult(potential_t, target):
        t_satisfied.append(potential_t)

    return t_satisfied

  else:

    return []

def check_sum(tuplev, target):

  sumval = 0
  for i in tuplev:
    sumval += i

  if sumval == target:
    return True
  else:
    return False 


def check_mult(tuplev, target):

  mval = 1
  for i in tuplev:
    mval = mval * i

  if mval == target:
    return True
  else:
    return False 

def check_sub(tuplev, target):

  for t in itertools.permutations(tuplev):
    sub = t[0]
    for s in range(1, len(t)):
      sub = sub - t[s]
    if sub == target:
      return True
  return False


def check_div(tuplev, target):

  for t in itertools.permutations(tuplev):
    div = t[0]
    #flag = 0
    for d in range(1, len(t)):
      div = div / t[d]
    if div == float(target):
      return True
  return False











              

