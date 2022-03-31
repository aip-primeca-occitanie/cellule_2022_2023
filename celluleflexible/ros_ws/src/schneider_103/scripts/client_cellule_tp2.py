#!/usr/bin/env python
# -*- coding: utf-8 -*-

# read_register
# read 10 registers and print result on stdout

# you can use the tiny modbus server "mbserverd" to test this code
# mbserverd is here: https://github.com/sourceperl/mbserverd

# the command line modbus client mbtget can also be useful
# mbtget is here: https://github.com/sourceperl/mbtget

from pyModbusTCP.client import ModbusClient
import rospy
from std_msgs.msg import String
from schneider_104.srv import *
flag = 0
SERVER_HOST = "192.168.0.104"
SERVER_PORT = 502
print("Adresse ip : {} port : {} ".format(SERVER_HOST, SERVER_PORT))
c = ModbusClient()
c.host(SERVER_HOST)
c.port(SERVER_PORT)
c.open()
print('Demande accès serveur en cours')
rospy.init_node('client_cellule_tp_104',anonymous=False)

def retour(req):
	if not c.is_open() :
		print('serveur non ouvert')
		c.open()
	if c.is_open():
		global flag			
		if flag == 0 :		
			print('serveur ouvert')
			flag = 1
		resp = Retour_celluleResponse()
		coils = c.read_coils(0, 76)
		if coils :
			resp.ST8 = coils[0]
			resp.ST9 = coils[1]
			resp.ST10 = coils[2]
			resp.ST11 = coils[3]
			resp.ST12 = coils[4]
			resp.ST13 = coils[5]
			resp.ST14 = coils[6]
			resp.ST15 = coils[7]
			resp.ST16 = coils[8]
			resp.ST17 = coils[9]
			resp.R5D = coils[10]
			resp.R6D = coils[11]
			resp.R7D = coils[12]
			resp.R8D = coils[13]
			resp.R5G = coils[14]
			resp.R6G = coils[15]
			resp.R7G = coils[16]
			resp.R8G = coils[17]
			resp.PI3 = coils[18]
			resp.PI4 = coils[19]
			resp.PI5 = coils[20]
			resp.PI6 = coils[21]
			resp.D5 = coils[22]
			resp.D6 = coils[23]
			resp.D7 = coils[24]
			resp.D8 = coils[25]
			resp.V5 = coils[26]
			resp.V6 = coils[27]
			resp.V7 = coils[28]
			resp.V8 = coils[29]
			resp.PS8 = coils[50]
			resp.PS9 = coils[51]
			resp.PS10 = coils[52]
			resp.PS11 = coils[53]
			resp.PS12 = coils[54]
			resp.PS13 = coils[55]
			resp.PS14 = coils[56]
			resp.PS15 = coils[57]
			resp.PS16 = coils[58]
			resp.PS17 = coils[59]
			resp.D5D = coils[60]
			resp.D6D = coils[61]
			resp.D7D = coils[62]
			resp.D8D = coils[63]
			resp.D5G = coils[64]
			resp.D6G = coils[65]
			resp.D7G = coils[66]
			resp.D8G = coils[67]
			resp.CPI3 = coils[68]
			resp.CPI4 = coils[69]
			resp.CPI5 = coils[70]
			resp.CPI6 = coils[71]
			resp.CP4 = coils[72]
			resp.CP5 = coils[73]
			resp.CP6 = coils[74]
			resp.CP7 = coils[75]
			return resp
		
	
	

def callback(data):
	trame = data.data.split(',')
	for i in range((len(trame))):
		if (i%2 != 0):
			if not c.is_open() :
				c.open()
			if c.is_open() :
				c.write_single_coil(int(trame[i]), int(trame[i+1]))
	

if __name__ == "__main__":
	rospy.Service('retour_cellule', Retour_cellule, retour)
	rospy.Subscriber("control_cellule", String, callback)
	rospy.spin()
	

c.close()
