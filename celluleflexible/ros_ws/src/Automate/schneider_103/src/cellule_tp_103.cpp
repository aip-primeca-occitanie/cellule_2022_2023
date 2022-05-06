#include "cellule_tp_103.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <schneider_103/Retour_cellule_103.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


Cellule_tp::Cellule_tp(ros::NodeHandle noeud)
{
	cmd_aigGauche_cell_103=noeud.subscribe("/commande/Simulation/AiguillageGauche",100,&Cellule_tp::AigGaucheCallback,this);
	cmd_aigDroite_cell_103=noeud.subscribe("/commande/Simulation/AiguillageDroite",100,&Cellule_tp::AigDroiteCallback,this);
	cmd_PS_103=noeud.subscribe("/commande/Simulation/Actionneurs_stops", 100,&Cellule_tp::CmdPSCallback,this);
	pub_103 = noeud.advertise<std_msgs::String>("/control_cellule_103", 1);
	cap_103 = noeud.advertise<schneider_103::Msg_SensorState>("/commande/Simulation/Capteurs", 1);
	client = noeud.serviceClient<schneider_103::Retour_cellule_103>("retour_cellule_103");
	choixMode = noeud.subscribe("/commande_locale/ChoixMode", 10,&Cellule_tp::TypeMode,this);
	mode = 0;
}
	
Cellule_tp::~Cellule_tp()
{
}

void Cellule_tp::TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1)
{
	mode = msg1->mode;
}


void Cellule_tp::read()
{	
	if(mode==1){
		srv.request.memoire = 1;
		std::this_thread::sleep_for (std::chrono::milliseconds(200));
		while(client.call(srv));
		SensorState.id = 103;
		SensorState.PS[7] = srv.response.PS7;
		SensorState.PS[18] = srv.response.PS18;
		SensorState.PS[19] = srv.response.PS19;
		SensorState.DD[3] = srv.response.D3D;
		SensorState.DD[4]= srv.response.D4D;
		SensorState.DD[9] = srv.response.D9D;
		SensorState.DD[10] = srv.response.D10D;
		SensorState.DG[3] = srv.response.D3G;
		SensorState.DG[4] = srv.response.D4G;
		SensorState.DG[9] = srv.response.D9G;
		SensorState.DG[10] = srv.response.D10G;
		SensorState.CP[3] = srv.response.CP3;
		SensorState.CP[8] = srv.response.CP8;	
		cap_103.publish(SensorState);
	}
}


void Cellule_tp::write(vector<vector<int>> consigne)
{
	data = "";
	for (int i=0; i<consigne.size(); i=i+1){
        data += "," + to_string(consigne[i][0]) + "," + to_string(consigne[i][1]);
    }
	msg.data = data;
	cout<<"data="<<data<<endl;
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
	pub_103.publish(msg);
}


void Cellule_tp::AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){
		ROS_INFO("On bouge a gauche, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==Aiguillage_::A3)
		{	
			this->write({{Actionneur_::D3, 1}, {Actionneur_::V3, 0},{Actionneur_::R3D, 0},{Actionneur_::R3G, 1}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A4)
		{	
			this->write({{Actionneur_::D4, 1}, {Actionneur_::V4, 0},{Actionneur_::R4D, 0},{Actionneur_::R4G, 1}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A9)
		{	
			this->write({{Actionneur_::D9, 1}, {Actionneur_::V9, 0},{Actionneur_::R9D, 0},{Actionneur_::R9G, 1}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A10)
		{	
			this->write({{Actionneur_::D10, 1}, {Actionneur_::V10, 0},{Actionneur_::R10D, 0},{Actionneur_::R10G, 1}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
	}
}

void Cellule_tp::AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){	
		ROS_INFO("On bouge a droite, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==Aiguillage_::A3)
		{	
			this->write({{Actionneur_::D3, 1}, {Actionneur_::V3, 0},{Actionneur_::R3D, 1},{Actionneur_::R3G, 0}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A4)
		{	
			this->write({{Actionneur_::D4, 1}, {Actionneur_::V4, 0},{Actionneur_::R4D, 1},{Actionneur_::R4G, 0}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A9)
		{	
			this->write({{Actionneur_::D9, 1}, {Actionneur_::V9, 0},{Actionneur_::R9D, 1},{Actionneur_::R9G, 0}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
		if (msg_aigs->data==Aiguillage_::A10)
		{	
			this->write({{Actionneur_::D10, 1}, {Actionneur_::V10, 0},{Actionneur_::R10D, 1},{Actionneur_::R10G, 0}});
			ros::Duration(2).sleep();
			this->write({{Actionneur_::V3, 1}, {Actionneur_::V4, 1}, {Actionneur_::V9, 1}, {Actionneur_::V10, 1}, {Actionneur_::D3, 0}, {Actionneur_::D4, 0}, {Actionneur_::D9, 0}, {Actionneur_::D10, 0} });
		}
	}
}


void Cellule_tp::CmdPSCallback(const commande_locale::Msg_StopControl actionneurs_simulation_Stop)
{
	if(mode==1){	
		int i;
		const int tabPS[10]={6,7,18,19};
		for(i=0;i<4;i++){
			if(actionneurs_simulation_Stop.STOP[tabPS[i]]==1){
				this->write({{i, 0}});  // Les actionneurs STi sont mis à 0
			}
			else
			{
				this->write({{i, 1}});
			}
		}
		for(i=0;i<4;i++){
			if(actionneurs_simulation_Stop.GO[tabPS[i]]==1){
				this->write({{i, 1}});
			}
			else
			{
				this->write({{i, 0}});
			}
		}
	}
}

