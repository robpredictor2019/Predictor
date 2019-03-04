## Prerequisites (http://www.simon-rohou.fr/research/tubex-lib/installation.html)


$ sudo apt-get install -y python2.7 flex bison gcc g++ make pkg-config

# installer Ibex :
	trouver ibex-2.6.0.tar.gz sur internet

	tar xvfz ibex-2.6.0.tar.gz
	cd ibex-2.6.0
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

##pour inclure les libraries : (A TESTER) 

- ds CmakeList de MOOS-extend/src/??
	TARGET_LINK_LIBRARIES( ${ibex tubex
							

- ds CmakeList de MOOS-extend, soit:
#=============================================================================
# Ibex
#=============================================================================

INCLUDE_DIRECTORIES("???/ibex-lib-ibex-2.6.0/__build__")
LINK_DIRECTORIES("???/ibex-lib-ibex-2.6.0/__build__/src/libibex.a")
#=============================================================================
# Tubex
#=============================================================================

INCLUDE_DIRECTORIES("tubex-lib/build/include")
LINK_DIRECTORIES("???/tubex-lib/build/src/tube/libtubex-core.a")

soit : faire des paths dans /.bashrc pour ne pas avoir les "???" à mettre en dur


##pour compiler le programme : 
- ds terminal 1 : vibes
- ds terminal 2 : ds tubex-lib/build ->$ make
- ds terminal 3 : ds tubex-lib/build/examples/cpp/test_sarah -> $ ./ex_test_sarah


##pour reccuperer intervals :
   //tube.sliceBox(index) -> ( [temps], [image], dt, nb_slices )
   //tube.size() -> nb interval
   //tube.domain() -> temps
   //tube[index] -> image
