import numpy as np
import math

import matplotlib as plot
lst1 = [2, 3, 4, 1, 1, 5]
lst1 = [0] * 5
lst1.append(1)

print(lst1)
print(sum(lst1))
lst1.pop(0)

print(lst1)
# print(sum([27.747757,  16.225527,   9.628392,  22.675572,  39.542556,  28.196912,  23.796117,   9.953100,    3.574894,  11.549032]))
lst2 = list(set(lst1))

# print(lst2)

lst2.pop()

# print(lst2)


a = np.zeros((1, 10), dtype = int)
a = a[0]
# print(a)

b = [ 97710.00398019, 102536.10027733, 84901.324553, 89062.12263411, 100149.62334384, 82998.2383425, 55320.88077253, 105632.97354116, 68949.7287128, 92236.15234625]
c = [ 33407.4330836, 36449.01580641, 151605.10963257, 113707.57215009, 55725.35565292, 100500.49792146, 138329.16649872, 96480.36491829, 143470.16640279, 213174.80001004]
# print(sum(b))

# print(sum(c))

a = math.atan2((400.0057919628309 - 399.92469390395235), (146.48465310097185 - 146.42115139186325))
print(a)
print(a/3.14*360)

# a = np.random.normal(0, 0.01, 1)[0]
# print(a)

print(np.random.normal(0, 0))

R_k = np.array([[1.0,   0,    0],
                             [  0, 1.0,    0],
                             [  0,    0, 1.0]])

# A = R_k
# for i in range(2-1):
#     A = np.concatenate((A, R_k), axis = 1)
# R_k = np.diag([1, 1, 0.04, 1, 1, 0.04]) 
# print(R_k)
# print(A)
# print(A @ R_k @ A.T)


