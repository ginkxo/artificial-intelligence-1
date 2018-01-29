#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newVar=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newVar (newly instaniated variable) is an optional argument.
      if newVar is not None:
          then newVar is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newVar = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp


      PROPAGATOR called with newVar = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.
   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
  '''Do forward checking. That is check constraints with 
      only one uninstantiated variable. Remember to keep 
      track of all pruned variable,value pairs and return '''
   #IMPLEMENT

  pruned = []

  if not newVar:
    for c in csp.get_all_cons():
      var = c.get_scope()
      if (len(var) == 1) and (c.get_n_unasgn() == 1):

        vals = [None]

        for assignment in var[0].cur_domain():
          var_tup = (var[0], assignment)
          vals[0] = assignment
          if not c.check(vals):
            var[0].prune_value(assignment)
            pruned.append(var_tup)
            if var[0].cur_domain_size() == 0:
              return False, pruned

    return True, pruned 
  # newVar is the most recently assigned variable!
  else: 
    for c in csp.get_cons_with_var(newVar): # these return constraint objects 
      if c.get_n_unasgn() == 1:

        vals = []
        unassigned_index_in_vars = 0
        i = -1
        variables = c.get_scope()
        unassigned_var = None

        for allvar in variables:
          vals.append(allvar.get_assigned_value()) # try printing vals after this later
          i += 1
          if allvar.get_assigned_value() == None:
            unassigned_index_in_vars = i 
            unassigned_var = allvar

        for assignment in unassigned_var.cur_domain():

          var_tup = (unassigned_var, assignment)
          vals[unassigned_index_in_vars] = assignment 
          #if (check(vals)):
          #  break -- should we break? i don't think so 
          if not c.check(vals):
            unassigned_var.prune_value(assignment)
            pruned.append(var_tup)
            if unassigned_var.cur_domain_size() == 0:
              return False, pruned
            # prune 
            # if domain size == 0, return false and pruned
    return True, pruned 


def prop_GAC(csp, newVar=None):
  '''Do GAC propagation. If newVar is None we do initial GAC enforce 
    processing all constraints. Otherwise we do GAC enforce with
    constraints containing newVar on GAC Queue'''
  #IMPLEMENT
  pruned = []
  gac_q = []

  if not newVar:

    for c1 in csp.get_all_cons():
      gac_q.append(c1)

  else:  

    for c2 in csp.get_cons_with_var(newVar):
      gac_q.append(c2)

  while gac_q: 

    c = gac_q.pop(0)
    for v in c.get_scope():
      for d in v.cur_domain():
        var_tup = (v, d)
        if not c.has_support(v, d):
          v.prune_value(d)
          pruned.append(var_tup)
          if v.cur_domain_size() == 0:
            gac_q = [] # will this empty?
            return False, pruned
          else:
            for c_x in csp.get_cons_with_var(v): 
              if c_x not in gac_q:
                gac_q.append(c_x)

  return True, pruned

