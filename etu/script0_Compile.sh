
if [ -z $1 ]
then
echo "the first input argument is missing ! please specify the name of prefix of your main command file (without .main_commande.cpp)"
exit
fi

if [ ! -f $1.main_commande.cpp ]
then
  echo the file $1.main_commande.cpp does not exists
  exit
fi

if [ -z $2 ]
then
echo "the second input argument is missing ! please specify the name of product configuration file (without .prodconfig.h)"
exit
fi

if [ ! -f dataprodconfig/$2.prodconfig.h ]
then
  echo the file $2.prodconfig.h does not exists in folder dataprodconfig
  exit
fi

cp $1.main_commande.cpp ../celluleflexible/ros_ws/src/commande/src/main_commande.cpp
cp dataprodconfig/$2.prodconfig.h ../celluleflexible/ros_ws/src/commande/src/prodconfig/productconfig.h
#source /opt/ros/melodic/setup.bash
cd ../celluleflexible/ros_ws
source devel/setup.bash
catkin_make
cd ../../etu
#../celluleflexible/./launch.sh commented to avoid execution





