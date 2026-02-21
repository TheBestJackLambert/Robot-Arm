#factorial function
def fac(x):
  y = 1
  for i in range(1, x+1):
    y *= i
  return y

#cosine function
def cos(x):
  x = wrap(x)
  y = 0
  for i in range(25):
    y += 1/(fac(2*i)) * x ** (2*i) * (-1)**i
  return y

#sine function
def sin(x):
  x = wrap(x)
  y = 0
  for i in range(25):
    y += 1/(fac(1 + 2*i)) * x ** (2*i + 1) * (-1)**i
  return y

#tan function
def tan(x):
  y = sin(x)/cos(x)
  return y

#arccos function
def arccos(x, y):
  b = abs((x**2-y**2))**.5/y
  c = arctan(y, b)
  return c

def wrap(a):
  a = (a + pi) % (2 * pi) - pi
  return a

#arctan function
def arctan(x, y):

  #if denominator is zero returns corresponding angle
  if x == 0:
    if y > 0: return (pi/2)
    if y < 0: return (-pi/2)
    return 0.0
  u = y / x

  #determines sign of angle
  sgn = 1.0 if u >= 0 else -1.0
  t = abs(u)
  inv = False

  #if y>x flips it or easier compute and signals for unflip later on
  if t > 1.0:
    inv = True
    t = 1.0 / t
  a = 0.0

  #does taylor series 20 times
  for i in range(20):
    a += ((-1)**i) * (t**(2*i + 1)) / (2*i + 1)

  #adjusts for sign
  a = sgn * ( (pi/2) - a ) if inv else sgn * a

  #puts in correct quadrant
  if x < 0 and y >= 0: a += pi
  elif x < 0 and y < 0: a -= pi
  return a


#approximates pi
pi = 0
for i in range(15000):
  pi += 4/(1 + 2*i) * (-1)**i

#converts radians to degrees
def rad(x):
  x *= 180/pi
  return x

#converts degrees to radians
def deg(x):
  x *= pi/180
  return x

#finds closest value to v in a list
def cv(l, v):
  for i in range(len(l)):
    l[i] = abs(l[i]-v)
  return min(l)

#checks if it's possible to reach the point
def isPossible(t, ll):
  abc = ll.copy()
  a = t[0]
  b = t[1]
  c = t[2]
  lengthdiff = max(abc)
  totallength = max(abc)
  abc.remove(max(abc))
  for i in range(len(abc)):
    if lengthdiff < 0:
      lengthdiff = lengthdiff + cv(abc, lengthdiff)
    else:
      lengthdiff = lengthdiff - cv(abc, lengthdiff)
    totallength += cv(abc, lengthdiff)
    abc.remove(cv(abc, lengthdiff))
  distance = (a**2+b**2+c**2)**0.5
  if distance > totallength:
    return False
  elif distance < lengthdiff:
    return False
  else:
    return True


def fk(length, angles, axis, lorient):
  M =[[1,0,0,0],
      [0,1,0,0],
      [0,0,1,0],
      [0,0,0,1]]
  for i in range(len(angles)):
    angle = angles[i]
    c = cos(angle)
    s = sin(angle)
    if axis[i] == "x":
      matrix = [[1, 0, 0, 0],
                [0, c, -s, 0],
                [0, s, c, 0],
                [0, 0, 0, 1]]
    elif axis[i] == "y":
      matrix = [[c, 0, s, 0],
                [0, 1, 0, 0],
                [-s, 0, c, 0],
                [0, 0, 0, 1]]
    elif axis[i] == "z":
      matrix = [[c, -s, 0, 0],
                [s, c, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]
    atrix = [[1,0,0,length[i]],
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]]
    shorb = mm(matrix, atrix)
    M = mm(M, shorb)
  return [M[0][3], M[1][3], M[2][3]]

def angle(angles, axis):
  shorb =[[1,0,0,0],
          [0,1,0,0],
          [0,0,1,0],
          [0,0,0,1]]
  for i in range(len(angles)):
    a = angles[i]
    if axis[i] == "x":
      matrix = [[1, 0, 0, 0],
                [0, cos(a), -sin(a), 0],
                [0, sin(a), cos(a), 0],
                [0, 0, 0, 1]]
    elif axis[i] == "y":
      matrix = [[cos(a), 0, sin(a), 0],
                [0, 1, 0, 0],
                [-sin(a), 0, cos(a), 0],
                [0, 0, 0, 1]]
    elif axis[i] == "z":
      matrix = [[cos(a), -sin(a), 0, 0],
                [sin(a), cos(a), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]
    shorb = mm(shorb, matrix)
  return [shorb[2][2], shorb[2][1], shorb[1][2]]


#finds distance from goal
def distance(coordL1, coordL2):
  squareSum = 0
  for i in range(3):
    squareSum+=(coordL1[i]-coordL2[i])**2
  return squareSum**(1/2)

def sphere(coordinates):
  k = 0
  for i in range(3):
    k += coordinates[i]**2
  z = coordinates[2]
  a = k ** .5
  b = arctan(coordinates[0], coordinates[1])
  c = arccos(z, (k - z **2)**.5)
  return [a, b, c]

def system(beeple):
  return [0, 0, 0]

#gets the world-space rotation axis for a given joint
def get_world_axis(angles, orientation, joint_idx):
  M = [[1,0,0],
       [0,1,0],
       [0,0,1]]
  for i in range(joint_idx):
    a = angles[i]
    c = cos(a)
    s = sin(a)
    if orientation[i] == "x":
      R = [[1,0,0],
           [0,c,-s],
           [0,s,c]]
    elif orientation[i] == "y":
      R = [[c,0,s],
           [0,1,0],
           [-s,0,c]]
    elif orientation[i] == "z":
      R = [[c,-s,0],
           [s,c,0],
           [0,0,1]]
    N = [[sum(M[r][k]*R[k][cc] for k in range(3)) for cc in range(3)] for r in range(3)]
    M = N
  if orientation[joint_idx] == 'x': col = 0
  elif orientation[joint_idx] == 'y': col = 1
  else: col = 2
  return [M[0][col], M[1][col], M[2][col]]

#finds the angle between 2 points, around a common center
def anglefind(length, angles, orientation, end, lorient, gorp):
  blorp = fk(length, angles, orientation, lorient)
  if gorp == 0:
    current = [0, 0, 0]
  else:
    blorple = []
    shlorb = []
    glorb = []
    plorb = []
    for i in range(gorp):
      blorple.append(length[i])
      shlorb.append(angles[i])
      glorb.append(orientation[i])
      plorb.append(lorient[i])
    current = fk(blorple, shlorb, glorb, plorb)
  axis_vec = get_world_axis(angles, orientation, gorp)
  return project(axis_vec, end, current, blorp)


def rotMat(angle, orient):
  if orient == "x":
    matrix = [[1, 0, 0, 0],
              [0, cos(angle), -sin(angle), 0],
              [0, sin(angle), cos(angle), 0],
              [0, 0, 0, 1]]
  elif orient == "y":
    matrix = [[cos(angle), 0, sin(angle), 0],
              [0, 1, 0, 0],
              [-sin(angle), 0, cos(angle), 0],
              [0, 0, 0, 1]]
  elif orient == "z":
    matrix = [[cos(angle), -sin(angle), 0, 0],
              [sin(angle), cos(angle), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]]
  return matrix

def project(axis_world, goal, current, joint):
  goal_vec = [goal[i] - current[i] for i in range(3)]
  end_vec = [joint[i] - current[i] for i in range(3)]

  #project onto plane perpendicular to rotation axis
  g_dot = sum(goal_vec[i]*axis_world[i] for i in range(3))
  e_dot = sum(end_vec[i]*axis_world[i] for i in range(3))
  gp = [goal_vec[i] - g_dot*axis_world[i] for i in range(3)]
  ep = [end_vec[i] - e_dot*axis_world[i] for i in range(3)]

  #signed angle via cross and dot product
  cross = sum(axis_world[i]*(ep[(i+1)%3]*gp[(i+2)%3] - ep[(i+2)%3]*gp[(i+1)%3]) for i in range(3))
  dot = sum(gp[i]*ep[i] for i in range(3))

  return arctan(dot, cross)


  #multiplies two matrices
def mm(a, b):
  c = []
  for i in range(len(a)):
    row = []
    for k in range(len(b)):
      z = 0
      for j in range(len(b[i])):
        z += a[i][j]*b[j][k]
      row.append(z)
    c.append(row)
  return c

#lists

aa = [0,0,0,0]
lorient = [0,0,0]

ll = [0.8, 1.2, 0.7, 0.6]
orient = ['z', 'x', 'y', 'z']
t = [0.9, 0.6, 1.1]

#does the inverse kinematics and finds the angles needed
def solver(t, ll):
  global aa
  error = .0001
  glorbulation = 0
  iterations = 10000
  if isPossible(t, ll) == False:
    return 'gorp you fricking glorper'
  dz = distance(t, fk(ll, aa, orient, lorient))
  while dz > error and iterations > glorbulation:
    for i in range(len(aa)):
      aa[len(aa)-i-1] += anglefind(ll, aa, orient, t, lorient, len(aa) - i - 1) * 1/len(aa)
    dz = distance(t, fk(ll, aa, orient, lorient))
    glorbulation += 1
    aa = list(map(wrap, aa))
    if glorbulation%1000 == 0:
      print(int(glorbulation/1000))
  print(fk(ll, aa, orient, lorient))
  return [rad(a) for a in aa]

print(solver(t, ll))
