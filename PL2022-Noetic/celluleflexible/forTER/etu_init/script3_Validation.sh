
if [ -z $1 ]
then
echo "the first input argument is missing ! please specify the name of prefix of your main command file (without .main_commande.cpp)"
exit
fi

if [ -z $2 ]
then
echo "the second input argument is missing ! please specify the name of product configuration file (without .prodconfig.h)"
exit
fi

if [ ! -f output/$1_$2.log.txt ]
then
  echo the file $1_$2.log.txt does not exists within folder output
  exit
fi

if [ ! -f dataprodconfig/$2.prodconfig.h ]
then
  echo the file $2.prodconfig.h does not exists within folder dataprodconfig
  exit
fi

python3 ../celluleflexible/forTER/checker.py dataprodconfig/$2.prodconfig.h output/$1_$2.log.txt  
