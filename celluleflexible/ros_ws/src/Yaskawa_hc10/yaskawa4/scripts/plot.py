import numpy  as np
import matplotlib.pyplot as plt
#from rospkg import on_ros_path
  
pos0 = []
vel0 = []
pos1 = []
pos2 =[]
pos3 = []
pos4 = []
pos5 = []
vel1 = []
vel2 = []
vel3 = []
vel4 = []
vel5 = []
time0 = []
time1 = []
acc0 = []
acc1 = []

position = np.loadtxt('pos.txt')
velocity = np.loadtxt('vel.txt')
acceleration = np.loadtxt('acc.txt')
position1 = np.loadtxt('pos1.txt')
velocity1 = np.loadtxt('vel1.txt')
acceleration1 = np.loadtxt('acc1.txt')
for i in range(0,len(position)):
    pos0.append(position[i][0])
    time0.append(i*0.05)
    vel0.append(velocity[i][0])
    acc0.append(acceleration[i][0])
for i in range(0,len(position1)):
    pos1.append(position1[i][0])
    time1.append(i*0.05)
    vel1.append(velocity1[i][0])
    acc1.append(acceleration1[i][0])
plt.plot(time0, pos0,'green')
plt.plot(time0, vel0,'blue')
plt.plot(time0,acc0, 'red')
plt.plot(time1, pos1,'g--')
plt.plot(time1, vel1,'b--')
plt.plot(time1,acc1, 'r--')


plt.show()