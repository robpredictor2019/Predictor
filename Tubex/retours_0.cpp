#include "Tube.h"
#include "VibesFigure_Tube.h"
#include "math.h"

using namespace std;
using namespace ibex;
using namespace tubex;

//base sur example 04


void displayCausalMap(const Tube& x, const Tube& y, int fig_x, int fig_y)
{
	const string fig_name = "Map (top view): [x](·)x[y](·)";
	const int slices_nb_to_display = 500;

	vibes::newFigure(fig_name);
	vibes::setFigureProperties(
		      vibesParams("figure", fig_name, "x", fig_x, "y", fig_y, "width", 900, "height", 600));
	vibes::axisLimits(-10, 150, -10, 30);

	// Robot's tubes projection
	int startpoint;
	for(int i=0 ; i<x.size() ; i+=max((int)(x.size()/slices_nb_to_display), 1))
	 	startpoint = i;

	for(int i=0 ; i<startpoint; i+=max((int)(x.size()/slices_nb_to_display), 1))
	{
		Interval intv_x = x[i];
		Interval intv_y = y[i];
		if(!intv_x.is_unbounded() && !intv_y.is_unbounded())
			vibes::drawBox(intv_x.lb(), intv_x.ub(), intv_y.lb(), intv_y.ub(),
		               "lightGray[white]", vibesParams("figure", fig_name));
	}
}

const Interval h(const IntervalVector& meas, const Tube& x, const Interval& x0, const Interval& y0)
{
	Interval angle_retours;
	
	//hypothese verite terrain:
	//angle_retours = Interval((178-1.15)*M_PI/180, (178+1.15)*M_PI/180); //angle 180
	
	//non VT
	Interval dist_x = x[meas[0].lb()] - x0;
	Interval dist_y = meas[1] - y0;
	angle_retours = M_PI + atan2(dist_y, dist_x);
	
	//cout <<" angle_ret deg : "<< 180/M_PI*angle_retours << endl;
	
	return angle_retours;
	
}

int main(int argc, char *argv[])
{
	//parametres robot
	Interval domain(0,28); //domain temps
	double timestep = 0.001;
	float speed = 10.;
	int alpha1 = 0, alpha2 = 0, alpha3 = 0;
	
	//initialisation des tubes commande
	Tube u(domain, timestep, Function("t", "-cos((t+33)/5)+[-0.02,0.02]"));
	const char *a0 = "[-0.02, 0.02]";
	const char *a45 = "[0.77, 0.81]";
	const char *a90 = "[1.55, 1.59]";
	const char *a135 = "[2.34, 2.38]";
	const char *a180 = "[3.12, 3.16]";
	const char *a225 = "[3.91, 3.95]";
	const char *a270 = "[4.69, 4.71]";
	const char *a315 = "[5.48, 5.52]";
	const char *a360 = "[6.26, 6.30]";
	Tube theta(domain, timestep, Function("t", a0));
	Interval domain2(double(7),domain.ub()); //domain temps
	Interval theta90(1.55, 1.59);
	theta.set(theta90, domain2); 
	Interval domain3(double(14),domain.ub()); //domain temps
	Interval theta_end;
	theta.set(theta_end, domain3); 
	
	//CI
	Interval x0 = Interval(0.).inflate(1.);
	Interval y0 = x0;
	Interval v0(speed-0.5, speed+0.5);
	Interval theta0 = Interval((-6./5.)*M_PI).inflate(0.02);
	
	//equations	
	Tube xdot = speed * cos(theta) + alpha1;
	Tube ydot = speed * sin(theta) + alpha2;
	Tube vdot = -v0 + u + alpha3;
	
	Tube x = xdot.primitive(x0);
	Tube y = ydot.primitive(y0);
	Tube v = vdot.primitive(v0);
	
	//slice à temps "GPS"	
   IntervalVector xtyt(2);
	xtyt[0] = x[14.000]; // [t]
   xtyt[1] = y[14.000]; // [y]
   
   //prise "GPS"	
	IntervalVector measurement(2);
	measurement[0] = Interval(14.000,14.001); // [t]
	double lb = xtyt[1].lb() + 0 * xtyt[1].diam()/3;
	double ub = xtyt[1].lb() + 1 * xtyt[1].diam()/3;
	measurement[1] = Interval(lb, ub); 	// [y] précision de l'incertitude
	//Interval domain3(measurement[0].lb(), domain.ub());//domain temps	
	bool contraction;
	int iter_cont = 0;
	do
	{
		Tube y_old = y;
		
		xdot &= speed * cos(theta) + alpha1; //se modifie
		ydot &= speed * sin(theta) + alpha2;
		
		// Applying a local contraction on y tube	
		y.ctcObs(ydot, measurement[0], measurement[1], false);
		theta.set(h(measurement, x, x0, y0), domain3);
		y.ctcFwd(ydot);
		x.ctcFwd(xdot);
		
		contraction = y != y_old; //contraction done
		y_old = y;
		iter_cont += 1;
		
	} while(contraction && iter_cont <= 1);
   
	//affichage
	//VibesFigure_Tube::show(&x, "Tube [x](·)", 100, 100);
	//VibesFigure_Tube::show(&y, "Tube [y](·)", 150, 120);
	//VibesFigure_Tube::show(&v, "Tube [v](·)", 150, 150);
	VibesFigure_Tube::show(&theta, "Tube [theta](·)", 150, 150);
	displayCausalMap(x, y, 200, 200);
	

	
	
	//cout
	int index_m = y.size()-1;
	int index_t = 0;
	
	
	/*
	cout <<"[x"<<index_t<<"][y"<<index_t<<"] : " << x[index_t] << y[index_t] <<endl;
   cout <<"[x"<<index_m<<"][y"<<index_m<<"] : " << x[index_m] << y[index_m] <<endl;

   cout <<"[x"<<14000<<"][y"<<14000<<"] : " << xtyt <<endl;
   cout <<"[x"<<14000<<"][y"<<14000<<"] : " << x[14000] << y[14000] <<endl;
	*/

	//show initial, resurface, final interval
   IntervalVector x0y0(2);
   x0y0[0] = x0; // [t]
   x0y0[1] = y0; // [y]
   IntervalVector xfinalyfinal(2);
   xfinalyfinal[0] = x[index_m]; // [t]
   xfinalyfinal[1] = y[index_m]; // [y]
   IntervalVector xgpsygps(2);
   xgpsygps[0] = x[measurement[0].lb()]; // [t]
   xgpsygps[1] = y[measurement[0].lb()]; // [y]
	
   vibes::drawBox(x0y0, "blue", vibesParams("figure", "Map (top view): [x](·)x[y](·)"));
   vibes::drawBox(xtyt, "green", vibesParams("figure", "Map (top view): [x](·)x[y](·)"));
   vibes::drawBox(xgpsygps, "green", vibesParams("figure", "Map (top view): [x](·)x[y](·)"));
   vibes::drawBox(xfinalyfinal, "red", vibesParams("figure", "Map (top view): [x](·)x[y](·)"));
  
   
   /* fonction reccup' interval
   tube.sliceBox(index) -> ( [temps], [image], dt, nb_slices )
   tube.size() -> nb interval
   tube.domain() -> temps
   tube[index] -> image
   */
   
   
	//fin 
	VibesFigure_Tube::endDrawing();
}
