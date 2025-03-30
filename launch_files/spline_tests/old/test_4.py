#%%
import sympy as sp
from IPython.display import display

t = sp.symbols('t')


#temporarily sets x to t
x = t
function_1 = (t/3)*sp.Matrix([[(1/2)*x**2],
                              [-x**2+3*x-(3/2)],
                              [(1/2)*(3-x)**2]])

display(sp.expand(function_1))

#sets x to t-1
x = t-1
function_2 = ((4-t)/3)*sp.Matrix([[(1/2)*x**2],
                                  [-x**2+3*x-(3/2)],
                                  [(1/2)*(3-x)**2]])

display(sp.expand(function_2))


#creates the complete function
complete_function = sp.Matrix([[function_1[0,0]],
                               [function_1[1,0] + function_2[0,0]],
                               [function_1[2,0] + function_2[1,0]],
                               [function_2[2,0]]])

display(sp.expand(complete_function))


# %%
