#include "cellule_tp_104.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <schneider_104/Retour_cellule_104.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


Cellule_tp::Cellule_tp(ros::NodeHandle noeud)
{
	cmd_aigGauche_cell_104=noeud.subscribe("/commande/Simulation/AiguillageGauche",100,&Cellule_tp::AigGaucheCallback,this);
	cmd_aigDroite_cell_104=noeud.subscribe("/commande/Simulation/AiguillageDroite",100,&Cellule_tp::AigDroiteCallback,this);
	cmd_PS_104=noeud.subscribe("/commande/Simulation/Actionneurs_stops", 100,&Cellule_tp::CmdPSCallback,this);
	pub_104 = noeud.advertise<std_msgs::String>("/control_cellule_104", 1);
	cap_104 = noeud.advertise<schneider_104::Msg_SensorState>("/commande/Simulation/Capteurs", 1);
	client = noeud.serviceClient<schneider_104::Retour_cellule_104>("retour_cellule_104");
	choixMode = noeud.subscribe("/commande_locale/ChoixMode", 10,&Cellule_tp::TypeMode,this);
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
	srv.request.memoire = 1;
	std::this_thread::sleep_for (std::chrono::milliseconds(200));
	if (client.call(srv))
	{
		

		SensorState.id = 104;
		SensorState.PS[8] = srv.response.PS8;
		SensorState.PS[9] = srv.response.PS9;
		SensorState.PS[10] = srv.response.PS10;
		SensorState.PS[11] = srv.response.PS11;
		SensorState.PS[12] = srv.response.PS12;
		SensorState.PS[13] = srv.response.PS13;
		SensorState.PS[14] = srv.response.PS14;
		SensorState.PS[15] = srv.response.PS15;
		SensorState.PS[16] = srv.response.PS16;
		SensorState.PS[17] = srv.response.PS17;
		SensorState.DD[5] = srv.response.D5D;
		SensorState.DD[6]= srv.response.D6D;
		SensorState.DD[7] = srv.response.D7D;
		SensorState.DD[8] = srv.response.D8D;
		SensorState.DG[5] = srv.response.D5G;
		SensorState.DG[6] = srv.response.D6G;
		SensorState.DG[7] = srv.response.D7G;
		SensorState.DG[8] = srv.response.D8G;
		SensorState.CPI[3] = srv.response.CPI3;
		SensorState.CPI[4] = srv.response.CPI4;
		SensorState.CPI[5] = srv.response.CPI5;
		SensorState.CPI[6] = srv.response.CPI6;
		SensorState.CP[4] = srv.response.CP4;
		SensorState.CP[5] = srv.response.CP5;
		SensorState.CP[6] = srv.response.CP6;
		SensorState.CP[7] = srv.response.CP7;
		
		if(mode==1){
			cap_104.publish(SensorState);
		}



	}
	else
	{
		read();
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
	pub_104.publish(msg);
}


void Cellule_tp::AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){
		ROS_INFO("On bouge a gauche, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==5)
		{	
			this->write({{22, 1}, {26, 0},{10, 0},{14, 1}});
			ros::Duration(2).sleep();
				this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==6)
		{	
			this->write({{23, 1}, {27, 0},{11, 0},{15, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==7)
		{	
			this->write({{24, 1}, {28, 0},{12, 0},{16, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==8)
		{	
			this->write({{25, 1}, {29, 0},{13, 0},{17, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
	}
}

void Cellule_tp::AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){	
		ROS_INFO("On bouge a droite, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==5)
		{	
			this->write({{22, 1}, {26, 0},{10, 1},{14, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==6)
		{	
			this->write({{23, 1}, {27, 0},{11, 1},{15, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==7)
		{	
			this->write({{24, 1}, {28, 0},{12, 1},{16, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==8)
		{	
			this->write({{25, 1}, {29, 0},{13, 1},{17, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
	}
}


void Cellule_tp::CmdPSCallback(const commande_locale::Msg_StopControl actionneurs_simulation_Stop)
{
	if(mode==1){	
		int i;
		const int tabPS[10]={8,9,10,11,12,13,14,15,16,17};
		for(i=0;i<10;i++){
			if(actionneurs_simulation_Stop.STOP[tabPS[i]]==1){
				this->write({{i, 0}});
			}
			else
			{
				this->write({{i, 1}});
			}
		}
		for(i=0;i<10;i++){
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

