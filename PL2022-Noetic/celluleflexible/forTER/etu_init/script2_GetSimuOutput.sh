
if [ -z $1 ]
then
echo "the first input argument is missing ! please specify the name of prefix of your main command file (without .main_commande.cpp)"
exit
fi

if [ ! -f ../celluleflexible/log.txt ]
then
  echo the log file does not exists. An error probably occured during the simulation !!!
  exit
fi

if [ -z $2 ]
then
echo "the second input argument is missing ! please specify the name of product configuration file (without .prodconfig.h)"
exit
fi

cp ../celluleflexible/log.txt output/$1_$2.log.txt





