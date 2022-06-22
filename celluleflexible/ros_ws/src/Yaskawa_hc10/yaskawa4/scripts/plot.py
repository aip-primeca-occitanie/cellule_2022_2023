import numpy as np
import matplotlib.pyplot as plt
#from rospkg import on_ros_path
  
pos0 = []
vel0 = []
pos1 = []
vel1 = []
t0 = []
t1 = []
acc0 = []
acc1 = []

position = np.loadtxt('pos.txt')
velocity = np.loadtxt('vel.txt')
acceleration = np.loadtxt('acc.txt')
time = np.loadtxt('time.txt')
position1 = np.loadtxt('pos1.txt')
velocity1 = np.loadtxt('vel1.txt')
acceleration1 = np.loadtxt('acc1.txt')
time1 = np.loadtxt('time1.txt')
bool = False
for i in range(0,len(position)):
    if i==0 :
        temp = 0.0
        bool = True
    elif ((i!=0) and (time[i][0] == 0 and time[i][1] == 0)):
        temp = t0[i-1]
        bool = True
    elif(bool == True):
        temp = t0[i-1] + (time[i][0]-time[i-1][0] + ((time[i][1] - time[i-1][1])*1e-9) )
        if ( (i != len(position)-1) and (time[i+1][0] == 0 and time[i+1][1] == 0)):
            bool = False
    t0.append(temp)
    vel0.append(velocity[i][0])
    acc0.append(acceleration[i][0])
    pos0.append(position[i][0])
bool = False
for i in range(0,len(position1)):
    if i==0 :
        temp1 = 0.0
        bool = True
    elif ((i!=0) and (time1[i][0] == 0 and time1[i][1] == 0)):
        temp1 = t1[i-1]
        bool = True
    elif(bool == True):
        temp1 = t1[i-1] + (time1[i][0]-time1[i-1][0] + ((time1[i][1] - time1[i-1][1])*1e-9) )
        if ( (i != len(position1)-1) and (time1[i+1][0] == 0 and time1[i+1][1] == 0)):
            bool = False
    pos1.append(position1[i][0])
    t1.append(temp1)
    vel1.append(velocity1[i][0])
    acc1.append(acceleration1[i][0])

fig, ((ax1, ax2, ax3)) = plt.subplots(3)
ax1.grid()
ax1.plot(t0, pos0,'green',label = 'OMPL')
ax1.plot(t1, pos1,'g--', label = 'PILZ')
# ax1.set_title('Position du joint0 pendant la trajectoire de pick and place')
ax1.set(xlabel='temps(s)', ylabel='position(rad)')
ax1.legend()
ax2.grid()
ax2.plot(t0, vel0,'blue', label = 'OMPL')
ax2.plot(t1, vel1,'b--', label = 'PILZ')
# ax2.set_title('Vitesse du joint0 pendant la trajectoire de pick and place')
ax2.set(xlabel='temps(s)', ylabel='vitesse(m/s)')
ax2.legend()
ax3.grid()
ax3.plot(t0,acc0, 'red', label = 'OMPL')
ax3.plot(t1,acc1, 'r--', label = 'PILZ')
# ax3.set_title('Acceleration du joint0 pendant la trajectoire de pick and place')
ax3.set(xlabel='temps(s)', ylabel='acceleration(m/s^2)')
ax3.legend()



plt.show()