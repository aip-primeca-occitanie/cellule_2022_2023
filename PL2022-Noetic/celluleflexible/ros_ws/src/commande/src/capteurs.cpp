/**** Bruno DATO, Abdellah ELGOURAIN, Evgeny SHULGA M1 EEA ISTR Université Paul Sabatier Toulouse III 2016 ****/

#include "capteurs.h"

Capteurs::Capteurs(ros::NodeHandle noeud)
{
	sub_capteurs_simu = noeud.subscribe("/commande/Simulation/Capteurs", 100, &Capteurs::Callback_capteurs_simulation,this);
	for(int i=0;i<25;i++) PSx[i]=0;
	for(int i=0;i<13;i++) DxD[i]=0;
	for(int i=0;i<13;i++) DxG[i]=0;
	for(int i=0;i<11;i++) CPx[i]=0;
	for(int i=0;i<9;i++) CPIx[i]=0;
}

Capteurs::~Capteurs()
{
}

void Capteurs::Callback_capteurs_simulation(const aiguillages::Msg_SensorState::ConstPtr& msg)
{
	if(msg->id==102){
		for(int i=1;i<7;i++) PSx[i]=msg->PS[i];
		for(int i=20;i<25;i++) PSx[i]=msg->PS[i];

		for(int i=1;i<3;i++) DxD[i]=msg->DD[i];
		for(int i=11;i<13;i++) DxD[i]=msg->DD[i];

		for(int i=1;i<3;i++) DxG[i]=msg->DG[i];
		for(int i=11;i<13;i++) DxG[i]=msg->DG[i];

		for(int i=1;i<3;i++) CPx[i]=msg->CP[i];
		for(int i=9;i<11;i++) CPx[i]=msg->CP[i];

		for(int i=1;i<3;i++) CPIx[i]=msg->CPI[i];
		for(int i=7;i<9;i++) CPIx[i]=msg->CPI[i];
	}
	else if(msg->id==103){
		PSx[7]=msg->PS[7];
		//manque PS6
		for(int i=18;i<20;i++) PSx[i]=msg->PS[i];

		for(int i=3;i<5;i++) DxD[i]=msg->DD[i];
		for(int i=9;i<11;i++) DxD[i]=msg->DD[i];

		for(int i=3;i<5;i++) DxG[i]=msg->DG[i];
		for(int i=9;i<11;i++) DxG[i]=msg->DG[i];

		CPx[3]=msg->CP[3];
		CPx[8]=msg->CP[8];
	}
	else if(msg->id==104){
		for(int i=8;i<18;i++) PSx[i]=msg->PS[i];
		for(int i=5;i<9;i++) DxD[i]=msg->DD[i];
		for(int i=5;i<9;i++) DxG[i]=msg->DG[i];
		for(int i=4;i<8;i++) CPx[i]=msg->CP[i];
		for(int i=3;i<7;i++) CPIx[i]=msg->CPI[i];
	}
	else{
		for(int i=0;i<25;i++) PSx[i]=msg->PS[i];
		for(int i=0;i<13;i++) DxD[i]=msg->DD[i];
		for(int i=0;i<13;i++) DxG[i]=msg->DG[i];
		for(int i=0;i<11;i++) CPx[i]=msg->CP[i];
		for(int i=0;i<9;i++) CPIx[i]=msg->CPI[i];
	}
}

bool Capteurs::get_PS(int num_PS)
{
	return PSx[num_PS];
}

bool Capteurs::get_DD(int num_DD)
{
	return DxD[num_DD];
}

bool Capteurs::get_DG(int num_DG)
{
	return DxG[num_DG];
}

bool Capteurs::get_CP(int num_CP)
{
	return CPx[num_CP];
}

bool Capteurs::get_CPI(int num_CPI)
{
	return CPIx[num_CPI];
}

bool MASK(long int registre,int numero_bit)
{
	return (bool)((registre & (long int)(pow(2,numero_bit)))/pow(2,numero_bit));
}
