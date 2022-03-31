#!/usr/bin/env python3
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
from schneider_103.srv import *
flag = 0
SERVER_HOST = "192.168.0.103"
print("adresse ip :")
print(SERVER_HOST)
SERVER_PORT = 502
rospy
print("port :")
print ('Port {}'.format(SERVER_PORT))
c = ModbusClient()
c.host(SERVER_HOST)
c.port(SERVER_PORT)
c.open()
print('Demande accès serveur')
rospy.init_node('client_cellule_tp_103',anonymous=False)

def retour(req):
	if not c.is_open() :
		print('Serveur non ouvert\n')
		c.open()
	if c.is_open():
		global flag
		if flag == 0 :		
			print('serveur ouvert\n')
			flag = 1
		resp = Retour_cellule_103Response()
		coils = c.read_coils(0, 75)
		print ("lecture de la mémoire automate")
		if coils :
			resp.ST6 = coils[0]
			resp.ST7 = coils[1]
			resp.ST18 = coils[2]
			resp.ST19 = coils[3]
			resp.R3D = coils[4]
			resp.R4D = coils[5]
			resp.R9D = coils[6]
			resp.R10D = coils[7]
			resp.R3G = coils[8]
			resp.R4G = coils[9]
			resp.R9G = coils[10]
			resp.R10G = coils[11]
			resp.D3 = coils[12]
			resp.D4 = coils[13]
			resp.D9 = coils[14]
			resp.D10 = coils[15]
			resp.V3 = coils[16]
			resp.V4 = coils[17]
			resp.V9 = coils[18]
			resp.V10 = coils[19]
			resp.PS7 = coils[20]
			resp.PS18 = coils[21]
			resp.PS19 = coils[22]
			resp.D3D = coils[23]
			resp.D4D = coils[24]
			resp.D9D = coils[25]
			resp.D10D = coils[26]
			resp.D3G = coils[27]
			resp.D4G = coils[28]
			resp.D9G = coils[29]
			resp.D10G = coils[30]
			resp.CP3 = coils[31]
			resp.CP8 = coils[32]
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
	rospy.Service('retour_cellule_103', Retour_cellule_103, retour)
	rospy.Subscriber("control_cellule_103", String, callback)
	rospy.spin()
	

c.close()
