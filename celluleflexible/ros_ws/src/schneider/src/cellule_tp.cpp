#include "cellule_tp.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <schneider/Retour_cellule.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


Cellule_tp::Cellule_tp(ros::NodeHandle noeud)
{	
	cmd_aigGauche_cell=noeud.subscribe("/commande/Simulation/AiguillageGauche",100,&Cellule_tp::AigGaucheCallback,this);
	cmd_aigDroite_cell=noeud.subscribe("/commande/Simulation/AiguillageDroite",100,&Cellule_tp::AigDroiteCallback,this);
	cmd_PS=noeud.subscribe("/commande/Simulation/Actionneurs_stops", 100,&Cellule_tp::CmdPSCallback,this);
	choixMode = noeud.subscribe("/commande_locale/ChoixMode", 10,&Cellule_tp::TypeMode,this);
	pub = noeud.advertise<std_msgs::String>("/control_cellule", 1);
	cap = noeud.advertise<schneider::Msg_SensorState>("/commande/Simulation/Capteurs", 1);
	client = noeud.serviceClient<schneider::Retour_cellule>("retour_cellule");
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
		/*ST1 = srv.response.ST1;
		ST2 = srv.response.ST2;
		ST3 = srv.response.ST3;
		ST4 = srv.response.ST4;
		ST5 = srv.response.ST5;
		ST20 = srv.response.ST20;
		ST21 = srv.response.ST21;
		ST22 = srv.response.ST22;
		ST23 = srv.response.ST23;
		ST24 = srv.response.ST24;
		R1D = srv.response.R1D;
		R2D = srv.response.R2D;
		R11D = srv.response.R11D;
		R12D = srv.response.R12D;
		R1G = srv.response.R1G;
		R2G = srv.response.R2G;
		R11G = srv.response.R11G;
		R12G = srv.response.R12G;
		PI1 = srv.response.PI1;
		PI2 = srv.response.PI2;
		PI7 = srv.response.PI7;
		PI8 = srv.response.PI8;
		D1 = srv.response.D1;
		D2 = srv.response.D2;
		D11 = srv.response.D11;
		D12 = srv.response.D12;
		V1 = srv.response.V1;
		V2 = srv.response.V2;
		V11 = srv.response.V11;
		V12 = srv.response.V12;
		PS1 = srv.response.PS1;
		PS2 = srv.response.PS2;
		PS3 = srv.response.PS3;
		PS4 = srv.response.PS4;
		PS5 = srv.response.PS5;
		PS20 = srv.response.PS20;
		PS21 = srv.response.PS21;
		PS22 = srv.response.PS22;
		PS23 = srv.response.PS23;
		PS24 = srv.response.PS24;
		D1D = srv.response.D1D;
		D2D = srv.response.D2D;
		D11D = srv.response.D11D;
		D12D = srv.response.D12D;
		D1G = srv.response.D1G;
		D2G = srv.response.D2G;
		D11G = srv.response.D11G;
		D12G = srv.response.D12G;
		CPI1 = srv.response.CPI1;
		CPI2 = srv.response.CPI2;
		CPI7 = srv.response.CPI7;
		CPI8 = srv.response.CPI8;
		CP1 = srv.response.CP1;
		CP2 = srv.response.CP2;
		CP9 = srv.response.CP9;
		CP10 = srv.response.CP10;*/

		SensorState.id = 102;
		SensorState.PS[1] = srv.response.PS1;
		SensorState.PS[2] = srv.response.PS2;
		SensorState.PS[3] = srv.response.PS3;
		SensorState.PS[4] = srv.response.PS4;
		SensorState.PS[5] = srv.response.PS5;
		SensorState.PS[6] = srv.response.PS6;
		SensorState.PS[20] = srv.response.PS20;
		SensorState.PS[21] = srv.response.PS21;
		SensorState.PS[22] = srv.response.PS22;
		SensorState.PS[23] = srv.response.PS23;
		SensorState.PS[24] = srv.response.PS24;
		SensorState.DD[1] = srv.response.D1D;
		SensorState.DD[2]= srv.response.D2D;
		SensorState.DD[11] = srv.response.D11D;
		SensorState.DD[12] = srv.response.D12D;
		SensorState.DG[1] = srv.response.D1G;
		SensorState.DG[2] = srv.response.D2G;
		SensorState.DG[11] = srv.response.D11G;
		SensorState.DG[12] = srv.response.D12G;
		SensorState.CPI[1] = srv.response.CPI1;
		SensorState.CPI[2] = srv.response.CPI2;
		SensorState.CPI[7] = srv.response.CPI7;
		SensorState.CPI[8] = srv.response.CPI8;
		SensorState.CP[1] = srv.response.CP1;
		SensorState.CP[2] = srv.response.CP2;
		SensorState.CP[9] = srv.response.CP9;
		SensorState.CP[10] = srv.response.CP10;
		if(mode==1){
			cap.publish(SensorState);
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
	pub.publish(msg);
}


void Cellule_tp::AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs)
{
	if(mode==1){
		ROS_INFO("On bouge a gauche, aig numero %d", msg_aigs->data);
		//commande type {Dx, Vx, RxD, RxG}
		if (msg_aigs->data==1)
		{	
			this->write({{22, 1}, {26, 0},{10, 0},{14, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==2)
		{	
			this->write({{23, 1}, {27, 0},{11, 0},{15, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==11)
		{	
			this->write({{24, 1}, {28, 0},{12, 0},{16, 1}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==12)
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
		if (msg_aigs->data==1)
		{	
			this->write({{22, 1}, {26, 0},{10, 1},{14, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==2)
		{	
			this->write({{23, 1}, {27, 0},{11, 1},{15, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==11)
		{	
			this->write({{24, 1}, {28, 0},{12, 1},{16, 0}});
			ros::Duration(2).sleep();
			this->write({{26, 1}, {27, 1}, {28, 1}, {29, 1}, {22, 0}, {23, 0}, {24, 0}, {25, 0} });
		}
		if (msg_aigs->data==12)
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
		const int tabPS[10]={1,2,3,4,5,20,21,22,23,24};
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

