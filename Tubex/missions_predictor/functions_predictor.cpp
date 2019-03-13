#include "Tube.h"
#include "VibesFigure_Tube.h"
#include "math.h"
#include <stdlib.h>

#include "functions_predictor.h"

using namespace std;
using namespace ibex;
using namespace tubex;


void displayCausalMap(const tubex::Tube& x, const tubex::Tube& y, int fig_x, int fig_y, const std::string& fig_name, const std::string& color)
{/*display tube x by tube y*/
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

void afficheWP(const std::vector<ibex::IntervalVector>& iles, const std::string& fig_name)
{
   /*show iles boxes*/
   vibes::drawBox(iles[0], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[1], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[2], "green[green]", vibesParams("figure", fig_name));   
}

void afficheWP(const std::vector<ibex::IntervalVector>& iles, const std::vector<ibex::IntervalVector>& zones_iles, const std::string& fig_name)
{
   /*show iles boxes*/
   vibes::drawBox(iles[0], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[1], "green[green]", vibesParams("figure", fig_name));
   vibes::drawBox(iles[2], "green[green]", vibesParams("figure", fig_name));
   
   /*show corona boxes*/
   vibes::drawBox(zones_iles[0], "green", vibesParams("figure", fig_name));
   vibes::drawBox(zones_iles[1], "green", vibesParams("figure", fig_name));
   vibes::drawBox(zones_iles[2], "green", vibesParams("figure", fig_name));    
}

void affichePOI(const tubex::Tube& x, const tubex::Tube& y, const std::vector<double>& t_iles, const ibex::IntervalVector& x0y0, const std::string& fig_name)
{
   /*show interest boxes*/
   IntervalVector boite_1(2);
	boite_1[0] = x[t_iles[0]]; // [t]
   boite_1[1] = y[t_iles[0]]; // [y]
	
   IntervalVector boite_2(2);
	boite_2[0] = x[t_iles[1]];
   boite_2[1] = y[t_iles[1]];
   
   IntervalVector boite_3(2);
   boite_3[0] = x[t_iles[2]];
   boite_3[1] = y[t_iles[2]];  
    
   //final box
   vibes::drawBox(boite_3, "red[red]", vibesParams("figure", fig_name));
   
   //initial box
   vibes::drawBox(x0y0, "blue[blue]", vibesParams("figure", fig_name));
  
}


const ibex::Interval calcul_angle(const ibex::IntervalVector& boite1, const ibex::IntervalVector& boite2)
{
	/*computes the new angle*/
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


double temps_visu_ile(int index_m, const tubex::Tube& x, const tubex::Tube& y, const ibex::IntervalVector& ile)
{
	/*get the first time the robot is in the island's viewable zone*/
	int t_ile = 0;
   int i = 0;
   
   while ((t_ile == 0) and (i<=index_m) )
   {
   	if ( (x[i].is_subset(ile[0]) != 0) and (y[i].is_subset(ile[1]) != 0) )
   	{
   		t_ile = i;
   		return t_ile;
   	}
   	i++;
   }
	return t_ile;
}

double temps_ile(int index_m, const tubex::Tube& x, const tubex::Tube& y, const ibex::IntervalVector& ile)
{
	/*get the first time the robot is centered around the island*/
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

void propagation(tubex::Tube& xf, tubex::Tube& yf, const ibex::Interval& domain, const double& timestep, const ibex::IntervalVector& boite, const ibex::IntervalVector& boite_cible, const int& alpha1, const int& alpha2, const int& speed, const std::string& fig_name)
{
	/* fait l'union de tubes venant de sous-intervales pris à t_gps*/
	
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

void egalite(tubex::Tube& x, tubex::Tube& x2, const double& t_f, const double& timestep)
{
	/*remplace les donnes de tube x2 dans tube x*/
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



