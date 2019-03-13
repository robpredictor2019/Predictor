#include "Tube.h"
#include "VibesFigure_Tube.h"
#include "math.h"

#include <stdlib.h>
//#include "functions_predictor.h"

using namespace std;
using namespace ibex;
using namespace tubex;

//base sur example 04


void displayCausalMap(const Tube& x, const Tube& y, int fig_x, int fig_y, const string& fig_name, const string& color)
{
	const int slices_nb_to_display = 500;

	vibes::setFigureProperties(
		      vibesParams("figure", fig_name, "x", fig_x, "y", fig_y, "width", 900, "height", 600));
	//vibes::axisLimits(-20,60,-20,120);
	
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
		               color, vibesParams("figure", fig_name));
	}
}

void afficheWP(const vector<IntervalVector>& iles, const string& fig_name)
{
   
   vibes::drawBox(iles[0], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[1], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[2], "green[green]", vibesParams("figure", fig_name));
   
   
}

void affichePOI(const Tube& x, const Tube& y, const vector<double>& t_iles, const IntervalVector& x0y0, const string& fig_name)
{
   
   IntervalVector boite_1(2);
	boite_1[0] = x[t_iles[0]]; // [t]
   boite_1[1] = y[t_iles[0]]; // [y]
	
   IntervalVector boite_2(2);
	boite_2[0] = x[t_iles[1]];
   boite_2[1] = y[t_iles[1]];
   
   IntervalVector boite_4(2);
   boite_4[0] = x[t_iles[3]];
   boite_4[1] = y[t_iles[3]];  
    
   //final box
   vibes::drawBox(boite_4, "red[red]", vibesParams("figure", fig_name));
   
   //initial box
   vibes::drawBox(x0y0, "blue[blue]", vibesParams("figure", fig_name));
  
}


const Interval calcul_angle(const IntervalVector& boite1, const IntervalVector& boite2)
{
	Interval angle_retours;
	
	Interval dist_x = boite1[0] - boite2[0];
	Interval dist_y = boite1[1] - boite2[1];
	angle_retours = M_PI + atan2(dist_y, dist_x);
	
	
	double lb = 2*atan(tan(angle_retours.lb()/2));
	double ub = 2*atan(tan(angle_retours.ub()/2));
	angle_retours = Interval(lb, ub);
		
	//cout <<" angle_ret deg : "<< 180/M_PI*angle_retours << endl;
	//cout <<" angle_ret rad : "<< angle_retours << endl;
	
	return angle_retours;
}


double temps_ile(int index_m, const Tube& x, const Tube& y, const IntervalVector& ile)
{
	
	int t_ile = 0;
   int i = 0;
     
   
   while ((t_ile == 0) and (i<=index_m) )
   {
   	double center_x = x[i].lb() + x[i].diam()/2;
		double center_y = y[i].lb() + y[i].diam()/2;
		
		Interval Icenter_x(x[i].lb() + x[i].diam()/2);
		Interval Icenter_y(y[i].lb() + y[i].diam()/2);
		
   	if ( (Icenter_x.is_subset(ile[0])) and (Icenter_y.is_subset(ile[1])) )
   	{
   		t_ile = i;
   		return t_ile;
   	}
   	i++;
   }
	return t_ile;
}

void propagation(Tube& xf, Tube& yf, const Interval& domain, const double& timestep, const IntervalVector& boite, const IntervalVector& boite_cible, const int& alpha1, const int& alpha2, const int& speed, const string& fig_name)
{
	
	
	IntervalVector sous_boite(2);
	bool contraction;
	int iter_cont = 0;
	int xyf_modif = 0;
	
	int nbx = int(boite[0].ub() - boite[0].lb());
	int nby = int(boite[1].ub() - boite[1].lb());
	
   for(int i=0;i<nbx;i++)
   {
		//interval [x]	
		double lbx = boite[0].lb() + i * boite[0].diam()/nbx;
		double ubx = boite[0].lb() + (i+1) * boite[0].diam()/nbx;
		sous_boite[0] = Interval(lbx, ubx);
		
		for(int j=0;j<nby;j++)
   	{
   		//interval [y]	
			double lby = boite[1].lb() + j * boite[1].diam()/nby;
			double uby = boite[1].lb() + (j+1) * boite[1].diam()/nby;
			//if (uby > boite[1].ub()) uby =  boite[1].ub();
			sous_boite[1] = Interval(lby, uby); 
	
			Tube x_n(domain, timestep);
			Tube y_n(domain, timestep);
			Tube dx_n(domain, timestep);
			Tube dy_n(domain, timestep);
			Tube theta_n(domain, timestep);
			
			//sous-intervales et propagation
			do
			{
				Tube y_old = y_n;
				
				theta_n.set(calcul_angle(sous_boite, boite_cible));
		
				dx_n &= speed * cos(theta_n) + alpha1; //se modifie
				dy_n &= speed * sin(theta_n) + alpha2;
		
				// Applying a local contraction on tubes
				x_n.ctcFwd(dx_n, sous_boite[0]);
				y_n.ctcFwd(dy_n, sous_boite[1]);
		
				contraction = y_n != y_old; //contraction done
				y_old = y_n;
				iter_cont += 1;
			} while(contraction && iter_cont <= 1);
			
			//cout << "angle : " << 180/M_PI*calcul_angle(sous_boite, boite_cible) << endl;
				
			// union des sous-tubes
			xf |= x_n;
			yf |= y_n;
		
			contraction = !contraction;
			iter_cont = 0;
			xyf_modif = 1;
		}
   }
}

void egalite(Tube& x, Tube& x2, const double& t_f, const double& timestep)
{
	double t_fin2 = x2.size()-1;
	double t_02 = 0;
	
	double t_fin = t_f/timestep;
	double t_0 = t_fin - t_fin2;
	
	if (t_0 < 0) t_0 = 0;
	//cout << "   time1 : "<< t_fin<<" "<<t_0 <<" time2 :"<< t_fin2 <<" "<< t_02 << endl;
	
	int i = t_0;
	int j = t_02;
	
	while ((i < t_fin) and (j <= t_fin2))
	{
		x.set(x2[j], i);
		i++;
		j++;
	}
}









int main(int argc, char *argv[])
{			
	//boites de position d'iles
	double ile_incert = 1.;
	IntervalVector ile1(2);
	ile1[0] = Interval(50.-ile_incert, 50.+ile_incert);
	ile1[1] = Interval(0.-ile_incert, 0.+ile_incert);
	IntervalVector ile2(2);
	ile2[0] = Interval(25.-ile_incert, 25.+ile_incert);
	ile2[1] = Interval(25.-ile_incert, 25.+ile_incert);
	IntervalVector ile3(2);
	ile3[0] = Interval(0.-ile_incert, 0.+ile_incert);
	ile3[1] = Interval(0.-ile_incert, 0.+ile_incert);
	
	//creation vecteur iles	
	vector<IntervalVector> iles;
	iles.push_back(ile3);
	iles.push_back(ile1);
	iles.push_back(ile2);
	iles.push_back(ile3);
		
	//parametres robot
	float speed = 1.;
	int alpha1 = 0, alpha2 = 0, alpha3 = 0;
	
	//CI
	Interval x0 = Interval(0.).inflate(0.5);
	Interval y0 = x0;
	Interval v0(speed-0.5, speed+0.5);
	Interval theta0 = Interval((-6./5.)*M_PI).inflate(0.02);
	
	//calcul theta initial
	IntervalVector x0y0(2);
   x0y0[0] = x0;
   x0y0[1] = y0;
	Interval angle_01 = calcul_angle(x0y0, ile1);
	

	//tube de commande theta
	Interval domain(0,150); //domain temporel
	double timestep = 0.001;
	Tube theta(domain, timestep);
	theta.set(angle_01, domain); 
	
	//equations	du mouvement
	Tube xdot = speed * cos(theta) + alpha1;
	Tube ydot = speed * sin(theta) + alpha2;
	Tube x = xdot.primitive(x0);
	Tube y = ydot.primitive(y0);
	
	//init figures vibe
	vibes::beginDrawing();
	const string fig_name = "Map1 : [xf](·)x[yf](·)";
	vibes::newFigure(fig_name);
	const string fig_name2 = "Map2 : [x](·)x[y](·)";
	const string fig_name3 = "Map3 : [x](·)x[y](·)";
	const string fig_name4 = "Map4 : [x](·)x[y](·)";
	
	
	
	
	
	
	
	
	
	
	double t_ile2, t_ile;
	int last_index = y.size()-1;
	
	
	// vecteur temps pour atteindre les iles, cree a la main
	vector<double> t_iles;
	t_iles.push_back(0.000);
	t_iles.push_back(48.999);
	t_iles.push_back(33.334);
	t_iles.push_back(33.964);
	
		

	for (int i=0; i<3; i++)
	{
		t_ile = temps_ile(last_index, x, y, iles[i])* timestep;
		cout << "temps pour trouver île "<<i<<" : " << t_ile << endl;
		
		if (t_ile == 0) t_ile = t_iles[i];
		
   	//slice a temps ou ile est trouvee
		IntervalVector boite_chgmt(2);
		boite_chgmt[0] = x[t_ile];
		boite_chgmt[1] = y[t_ile];
		
		//domaine sur lequel theta change, a partir de boite_chgmt
		Interval domain_i;
		if (t_ile == 0)
		{ 
			domain_i = Interval(double(t_ile), double(t_ile-0*timestep+t_iles[i+1]));
		}
		else
		{
			domain_i = Interval(double(t_ile), double(t_ile+2*timestep+t_iles[i+1]));
		}
		
		//changement des theta
		Tube xf(domain_i, timestep, Interval::EMPTY_SET);
		Tube yf(xf);
		propagation(xf, yf, domain_i, timestep, boite_chgmt, iles[i+1], alpha1, alpha2, speed, fig_name);
		
 		//calcul temps pour atteindre prochaine ile
		t_ile2 = temps_ile(domain_i.diam()/timestep, xf, yf, iles[i+1])* timestep;
		cout << "  temps île "<<i+1<<" vue : " << t_ile2 << endl;
		if (t_ile2 == 0) t_ile2 = t_iles[i+1];
		 
   	//remplace les valeurs dans x et y
		egalite(x, xf, t_ile+t_ile2, timestep);
		egalite(y, yf, t_ile+t_ile2, timestep);
				
		t_iles[i] = t_ile;
		
		//displayCausalMap(xf, yf, 0, 0, fig_name, "black[white]");
		
	}
	t_iles[3] = t_ile+t_ile2;
	
	cout << "temps total : " << t_ile+t_ile2 << endl << endl;
	
	//affichage
	
	vibes::newFigure(fig_name2);
	displayCausalMap(x, y, 200, 200, fig_name2, "gray[lightGray]");
	affichePOI(x, y, t_iles, x0y0, fig_name2);
	afficheWP(iles, fig_name2);
	






	
	//new initial positional box and yaw to island
	IntervalVector x1y1(2);
   x1y1[0] = x[t_iles[3]];
   x1y1[1] = y[t_iles[3]];
	angle_01 = calcul_angle(x1y1, ile1); //yaw

	//tube de commande theta
	theta.set(angle_01, domain); 
	xdot = speed * cos(theta) + alpha1;
	ydot = speed * sin(theta) + alpha2;

	Tube x2 = xdot.primitive(x[t_iles[2]]);
	Tube y2 = ydot.primitive(y[t_iles[2]]);
	
	//new times
	t_iles.push_back(0.000);
	t_iles.push_back(32.516);
	t_iles.push_back(32.339);
	t_iles.push_back(33.735);
	
	int i_temps;
	for (int i=0; i<3; i++)
	{
		//indice pour vecteur temps
		i_temps = 4+i;
	
		t_ile = temps_ile(last_index, x2, y2, iles[i])* timestep;
		cout << "temps pour trouver île "<<i<<" : " << t_ile << endl;
		
		if (t_ile == 0) t_ile = t_iles[i_temps];
		
   	//slice a temps ou ile est trouvee
		IntervalVector boite_chgmt(2);
		boite_chgmt[0] = x2[t_ile];
		boite_chgmt[1] = y2[t_ile];
		
		//domaine sur lequel theta change, a partir de boite_chgmt
		Interval domain_i;
		if (t_ile == 0)
		{ 
			domain_i = Interval(double(t_ile), double(t_ile-0*timestep+t_iles[i_temps+1]));
		}
		else
		{
			domain_i = Interval(double(t_ile), double(t_ile+2*timestep+t_iles[i_temps+1]));
		}
		
		//changement des theta
		Tube xf(domain_i, timestep, Interval::EMPTY_SET);
		Tube yf(xf);
		propagation(xf, yf, domain_i, timestep, boite_chgmt, iles[i+1], alpha1, alpha2, speed, fig_name);
		
 		//calcul temps pour atteindre prochaine ile
		t_ile2 = temps_ile(domain_i.diam()/timestep, xf, yf, iles[i+1])* timestep;
		cout << "  temps île "<<i+1<<" vue : " << t_ile2 << endl;
		if (t_ile2 == 0) t_ile2 = t_iles[i_temps+1];
		 
   	//remplace les valeurs dans x et y
		egalite(x2, xf, t_ile+t_ile2, timestep);
		egalite(y2, yf, t_ile+t_ile2, timestep);
				
		t_iles[i_temps] = t_ile;
		
		//displayCausalMap(xf, yf, 0, 0, fig_name, "black[white]");
		
	}
	t_iles[i_temps] = t_ile+t_ile2;

	cout << "temps total : " << t_ile+t_ile2 << endl << endl;
	
	//affichage
	vibes::newFigure(fig_name3);   	
   displayCausalMap(x2, y2, 0, 0, fig_name3, "gray[lightGray]");
   affichePOI(x2, y2, t_iles, x1y1, fig_name3);	
	afficheWP(iles, fig_name3);
	
	
	
	/*
	
	
	IntervalVector x2y2(2);
   x2y2[0] = x2[t_iles[5]];
   x2y2[1] = y2[t_iles[5]];
	angle_01 = calcul_angle(x2y2, ile1);

	//tube de commande theta
	theta.set(angle_01, domain); 
	xdot = speed * cos(theta) + alpha1;
	ydot = speed * sin(theta) + alpha2;

	Tube x3 = xdot.primitive(x2[t_iles[5]]);
	Tube y3 = ydot.primitive(y2[t_iles[5]]);
	
	t_iles.push_back(0.00);
	t_iles.push_back(48.33);
	t_iles.push_back(33.41);
	t_iles.push_back(33.83);
	
	for (int i=0; i<3; i++)
	{
		//indice pour vecteur temps
		i_temps = 7+i;
	
		t_ile = temps_ile(last_index, x3, y3, iles[i])* timestep;
		cout << "temps pour trouver île "<<i<<" : " << t_ile << endl;
		
		if (t_ile == 0) t_ile = t_iles[i_temps];
		
   	//slice a temps ou ile est trouvee
		IntervalVector boite_chgmt(2);
		boite_chgmt[0] = x3[t_ile];
		boite_chgmt[1] = y3[t_ile];
		
		//domaine sur lequel theta change, a partir de boite_chgmt
		Interval domain_i;
		if (t_ile == 0)
		{ 
			domain_i = Interval(double(t_ile), double(t_ile-0*timestep+t_iles[i_temps+1]));
		}
		else
		{
			domain_i = Interval(double(t_ile), double(t_ile+2*timestep+t_iles[i_temps+1]));
		}
		
		//changement des theta
		Tube xf(domain_i, timestep, Interval::EMPTY_SET);
		Tube yf(xf);
		propagation(xf, yf, domain_i, timestep, boite_chgmt, iles[i+1], alpha1, alpha2, speed, fig_name);
		
 		//calcul temps pour atteindre prochaine ile
		t_ile2 = temps_ile(domain_i.diam()/timestep, xf, yf, iles[i+1])* timestep;
		cout << "  temps île "<<i+1<<" vue : " << t_ile2 << endl;
		if (t_ile2 == 0) t_ile2 = t_iles[i_temps+1];
		 
   	//remplace les valeurs dans x et y
		egalite(x3, xf, t_ile+t_ile2, timestep);
		egalite(y3, yf, t_ile+t_ile2, timestep);
				
		t_iles[i_temps] = t_ile;
		
		//displayCausalMap(xf, yf, 0, 0, fig_name, "black[white]");
		
	}
	t_iles[i_temps] = t_ile+t_ile2;

	cout << "temps total : " << t_ile+t_ile2 << endl << endl;
		
	//affichage
	vibes::newFigure(fig_name4);
	displayCausalMap(x3, y3, 0, 0, fig_name4, "gray[lightGray]");
   affichePOI(x3, y3, t_iles, x2y2, fig_name4);	
	afficheWP(iles, fig_name4);
	
	
	*/
	
	
	/*
	
	Tube xfin(domain, timestep);
	Tube yfin(domain, timestep);
	for(int i=0;i<=2; i++)
	{
		double j1 = t_iles[i]/timestep;
		double j2 = t_iles[i+3]/timestep;
		cout << "min "<<i<<" " <<min(j1, j2) << endl;
		for (int j=0; j<=min(j1, j2); j++)
		{
			xfin.set((x[j] | x2[j]), j);
			yfin.set((y[j] | y2[j]), j);
		}
		//for (int i=mi(j1, j2)+1; j< max(
	}*/

	//affichage
	//displayCausalMap(x3, y3, 200, 200, fig_name2, "black[darkGray]");
	
	
	//cout << x.volume() << y.volume() << x.dist(x2) << y.dist(y2) << endl;
	//cout << x2.volume() << y2.volume() << x2.dist(x3) << y2.dist(y3) << endl << x3.volume() << y3.volume() << endl;
	//afficheWP(iles, fig_name2);
	
	//fin 
	VibesFigure_Tube::endDrawing();
}
