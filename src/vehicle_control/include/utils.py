#!/usr/bin/env python

import numpy as np


def get_interp(x_vals, y_vals, thresholds = None):

   if thresholds is None:
      return lambda x: np.interp(x, x_vals, y_vals)
   
   x_expanded, y_expanded = [], []
   for x,y,t in zip(x_vals, y_vals, thresholds):
      if t:
         x = [x-t/2, x, x+t/2]
         y = [y, y, y]
      else:
         x, y = [x], [y]

      x_expanded.extend(x)
      y_expanded.extend(y)

      
   return lambda x: np.interp(x, x_expanded, y_expanded)