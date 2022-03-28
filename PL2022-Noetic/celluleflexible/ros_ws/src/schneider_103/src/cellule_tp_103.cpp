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

		
		if(mode==1){
			cap_103.publish(SensorState);
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
	pub_103.publish(msg);
}


void Cellule_tp::AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){
		ROS_INFO("On bouge a gauche, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==3)
		{	
			this->write({{12, 1}, {16, 0},{4, 0},{8, 1}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==4)
		{	
			this->write({{13, 1}, {17, 0},{5, 0},{9, 1}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==9)
		{	
			this->write({{14, 1}, {18, 0},{6, 0},{10, 1}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==10)
		{	
			this->write({{15, 1}, {19, 0},{7, 0},{11, 1}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
	}
}

void Cellule_tp::AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){	
		ROS_INFO("On bouge a droite, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==3)
		{	
			this->write({{12, 1}, {16, 0},{4, 1},{8, 0}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==4)
		{	
			this->write({{13, 1}, {17, 0},{5, 1},{9, 0}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==9)
		{	
			this->write({{14, 1}, {18, 0},{6, 1},{10, 0}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
		}
		if (msg_aigs->data==10)
		{	
			this->write({{15, 1}, {19, 0},{7, 1},{11, 0}});
			ros::Duration(2).sleep();
			this->write({{16, 1}, {17, 1}, {18, 1}, {19, 1}, {12, 0}, {13, 0}, {14, 0}, {15, 0} });
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
				this->write({{i, 0}});
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

