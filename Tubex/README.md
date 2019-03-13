## Prerequisites (http://www.simon-rohou.fr/research/tubex-lib/installation.html)


$ sudo apt-get install -y python2.7 flex bison gcc g++ make pkg-config

# installer Ibex :
	tar xvfz ibex-lib-ibex-2.6.0.tar.gz
	cd ibex-lib-ibex-2.6.0
	./waf configure
	sudo ./waf install
	
# installer Tubex :
	git clone https://github.com/SimonRohou/tubex-lib
	cd tubex-lib
	./build.sh
	pip install Sphinx
	pip install sphinx_rtd_theme
	cd doc
	make html
	cd ../build
	cmake -DBUILD_TESTS=ON ..
	make
	make test

## Pour utiliser les codes:
- dans tubex-lib/examples/cpp mettre le dossier missions_predictor
- ajouter la ligne "add_subdirectory(missions_predictor)" dans le CMakeList.txt de tubex-lib/examples/cpp

## Pour compiler le programme mission triangle: 
- renommer triangle.cpp en main.cpp
- ds terminal 1 : vibes
- ds terminal 2 : ds tubex-lib/build ->$ make
- ds terminal 3 : ds tubex-lib/build/examples/cpp/test_sarah -> $ ./ex_test_sarah

## Pour compiler le programme mission aller-retour: 
- renommer retour_0.cpp en main.cpp
- ds terminal 1 : vibes
- ds terminal 2 : ds tubex-lib/build ->$ make
- ds terminal 3 : ds tubex-lib/build/examples/cpp/test_sarah -> $ ./ex_test_sarah

##pour reccuperer intervals :
   //tube.sliceBox(index) -> ( [temps], [image], dt, nb_slices )
   //tube.size() -> nb interval
   //tube.domain() -> temps
   //tube[index] -> image

