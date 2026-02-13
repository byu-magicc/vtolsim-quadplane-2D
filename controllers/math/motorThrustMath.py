#%%

import sympy as sp
from IPython.display import display


#creates the constants needed for this

a_0 = sp.symbols('a_0')
b_0, b_1 = sp.symbols('b_0, b_1')
c_0, c_1, c_2 = sp.symbols('c_0, c_1, c_2')

aa_0 = sp.symbols('aa_0')
bb_0 = sp.symbols('bb_0')
cc_0 = sp.symbols('cc_0')

#defines the two variables
Va = sp.symbols('V_a')
delta_t = sp.symbols('delta_t')


#defines a, b, and c
a = a_0
b = b_0*Va + b_1
c = c_0*(Va**2) + c_1*delta_t + c_2

#defines the seconds
aa = aa_0
bb = bb_0 * Va
cc = cc_0 * (Va**2)

#defines omega
omega = (-b + sp.sqrt(b**2 - 4*a*c))/(2*a)



#defines the thrust
Tp = aa * (omega**2) + bb * omega + cc


display(Tp)




# %%
