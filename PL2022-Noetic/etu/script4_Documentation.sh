
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


if [ ! -f ../celluleflexible/forTER/XXX_XXX_Doxyfile_config ]
then
  echo the file ../celluleflexible/forTER/XXX_XXX_Doxyfile_config does not exists ... this should not happen ... maybe a bug happened during installation of the TER
  exit
fi


cp ../celluleflexible/forTER/XXX_XXX_Doxyfile_config output/$1.doxyfileconfig.txt

sed -i 's/XXX_XXX/$1/g' output/$1.doxyfileconfig.txt


if [ ! -f output/$1.doxyfileconfig.txt ]
then
  echo the file $1.doxyfileconfig.txt does not exists within folder output
  exit
fi


if ! type "doxygen" > /dev/null; then
  # install foobar here
    if ! type "./doxygen" > /dev/null; then
          echo "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
          echo Le logiciel doxygen ne semble pas exister sur votre machine. Il ne sera pas possible de générer la doc avec ce script.
          echo "\n"
          echo Cependant, si vous connaissez le chemin où se trouve l_executable de doxygen, vous pouvez l executer et lui donner en entree output/$1.doxyfileconfig.txt directement dans le terminal de la manière suivante:
          echo "\t CHEMIN_VERS_DOSSIER_DOXYGEN/doxygen output/"$1".doxyfileconfig.txt"
          echo ou
          echo "\t CHEMIN_VERS_DOSSIER_DOXYGEN/./doxygen output/"$1".doxyfileconfig.txt"
          echo "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"
          exit
    fi
fi

 
doxygen output/$1.doxyfileconfig.txt






