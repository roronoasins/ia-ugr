#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>
#include <unistd.h>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores) {
	Action sig_accion = actIDLE;

	actual.fila        = sensores.posF;
	actual.columna     = sensores.posC;
	actual.orientacion = sensores.sentido;

	n_destinos = sensores.num_destinos;
	//for(int i=0; i < n_destinos; ++i)
	//{
		//goals.push_back(pair<int,int>(sensores.destino[0],sensores.destino[0]));
//	}

	cout << "Fila: " << actual.fila << endl;
	cout << "Col : " << actual.columna << endl;
	cout << "Ori : " << actual.orientacion << endl;

	// Capturo los destinos
	cout << "sensores.num_destinos : " << sensores.num_destinos << endl;
	objetivos.clear();
	for (int i=0; i<sensores.num_destinos; i++){
		estado aux;
		aux.fila = sensores.destino[2*i];
		aux.columna = sensores.destino[2*i+1];
		objetivos.push_back(aux);
	}

	if(!hay_plan)
	{
		hay_plan = pathFinding (sensores.nivel, actual, objetivos, plan);
	}


	if (hay_plan && (sensores.destino[0] != destino.fila or sensores.destino[1] != destino.columna) && (sensores.nivel == 0 || sensores.nivel == 1  || sensores.nivel == 2))
	{
		cout << "El destino ha cambiado\n";
		hay_plan = false;
	}

	if(hay_plan && plan.size() > 0)
	{
		sig_accion = plan.front();
		plan.erase(plan.begin());

	}else{
		cout << "No se pudo encontrar un plan" << endl;
	}

  return sig_accion;
}


// Llama al algoritmo de busqueda que se usara en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (int level, const estado &origen, const list<estado> &destino, list<Action> &plan){
	switch (level){
		case 0:
		{
			cout << "Demo\n";
			estado un_objetivo;
			un_objetivo = objetivos.front();
			cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
      return pathFinding_Profundidad(origen,un_objetivo,plan);
			break;
		}
		case 1:
		{
			cout << "Optimo numero de acciones\n";
			estado objetivo_anchura;
			objetivo_anchura = objetivos.front();
			cout << "fila: " << objetivo_anchura.fila << " col:" << objetivo_anchura.columna << endl;
			return pathFinding_Anchura(origen,objetivo_anchura,plan);
			break;
		}
		case 2:
		{
			cout << "Optimo en coste 1 Objetivo - A*\n";
			estado objetivo_astar;
			objetivo_astar = objetivos.front();
			this->destino = objetivo_astar;
			cout << "fila: " << objetivo_astar.fila << " col:" << objetivo_astar.columna << endl;
			return pathFinding_Astar(origen,objetivo_astar,plan);
			break;
		}
		case 3:
		{
			cout << "Optimo en coste 3 Objetivos - A*\n";
			list<estado> objetivos_Astar = objetivos;
			list<estado>::iterator it=objetivos_Astar.begin();
			for(it; it != objetivos_Astar.end(); ++it)
			{
				cout << (*it).fila << " " << (*it).columna << endl;
				goals.push_back(*it);
			}

			return pathFinding_Astar_multi(origen, objetivos_Astar, plan);
			break;
		}
		case 4:
		{
			cout << "reto" << endl;
							break;
		}
	}
	return false;
}


//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el codigo en caracter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla){
	if (casilla=='P' or casilla=='M')
		return true;
	else
	  return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
	int fil=st.fila, col=st.columna;

  // calculo cual es la casilla de delante del agente
	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil<0 or fil>=mapaResultado.size()) return true;
	if (col<0 or col>=mapaResultado[0].size()) return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col])){
		// No hay obstaculo, actualizo el parametro st poniendo la casilla de delante.
    st.fila = fil;
		st.columna = col;
		return false;
	}
	else{
	  return true;
	}
}

struct nodo{
	estado st;
	list<Action> secuencia;
	int f, g, h;
	bool zapatillas, bikini;
	int actual_goal;
	int dest_reached;
};

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion))
			return true;
		else
			return false;
	}
};

// Implementación de la busqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
	stack<nodo> Abiertos;								 // Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	Abiertos.push(current);

  while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		Abiertos.pop();
		Cerrados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			Abiertos.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			Abiertos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (Cerrados.find(hijoForward.st) == Cerrados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				Abiertos.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty()){
			current = Abiertos.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}


// Búsqueda en anchura
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
	queue<nodo> Abiertos;								 // Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	Abiertos.push(current);

  while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		Abiertos.pop();
		Cerrados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			Abiertos.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			Abiertos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (Cerrados.find(hijoForward.st) == Cerrados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				Abiertos.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la Abiertos
		if (!Abiertos.empty()){
			current = Abiertos.front();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}

// Búsqueda A*

class open_comparison
{
public:
  open_comparison() {}
  bool operator() (const nodo& n1, const nodo& n2) const
  {
		return n1.f > n2.f ;
  }
};

int coste(const nodo& nodo, const char &terreno, const string &accion)
{
	if(accion == "forward")
	{
		switch (terreno) {
			case 'A':
				return (nodo.bikini) ? 10 : 200;
			case 'B':
				return (nodo.zapatillas) ? 15 : 100;
			case 'T':
				return 2;
			default:
				return 1;
		}
	}else if(accion == "t_right" || accion == "t_left")
	{
		switch (terreno) {
			case 'A':
				return (nodo.bikini) ? 5 : 500;
			case 'B':
				return (nodo.zapatillas) ? 1 : 3;
			case 'T':
				return 2;
			default:
				return 1;
		}
	}
	return 0;
}

int ComportamientoJugador::DistanciaMH(const estado& x, const estado& y)
{
	return (int) (abs(x.fila - y.fila) + abs(x.columna - y.columna));
}

void checkEquipment(nodo& nodo, const char& celda)
{
	if (celda == 'K')
	 	nodo.bikini = true;
	else if (celda == 'D')
		nodo.zapatillas = true;
}

bool ComportamientoJugador::pathFinding_Astar(const estado &origen, const estado &destino, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan A*\n";
	plan.clear();
	set<estado,ComparaEstados> Cerrados; 														// Lista de Cerrados
	priority_queue<nodo, vector<nodo>, open_comparison> Abiertos;		// Lista de Abiertos
	nodo mejor_padre;
	vector<nodo> hijos;

	nodo current;
	current.st = origen;
	checkEquipment(current, mapaResultado[current.st.fila][current.st.columna]);
	current.g = coste(current, mapaResultado[current.st.fila][current.st.columna], "idle");
	//cout << "nodo actual -> fila: " << current.st.fila << ", columna: " << current.st.columna << ". Bikini: " << current.bikini << ", zapatillas: " << current.zapatillas << endl;
	current.h = DistanciaMH(current.st, destino);
	current.f = current.g + current.h;
	current.secuencia.empty();

	Abiertos.push(current);

	while (!Abiertos.empty() and (current.st.fila != destino.fila or current.st.columna != destino.columna))
	{
		Abiertos.pop();
		Cerrados.insert(current.st);
		//cout << "nodo actual -> fila: " << current.st.fila << ", columna: " << current.st.columna << endl;
		// se expande dicho nodo
		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		checkEquipment(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna]);
		//cout << "nodo derecha -> fila: " << hijoTurnR.st.fila << ", columna: " << hijoTurnR.st.columna << ". Bikini: " << hijoTurnR.bikini << ", zapatillas: " << hijoTurnR.zapatillas << endl;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		//cout << hijoTurnR.g << endl;
		hijoTurnR.g += coste(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna], "t_right");
		if(hijoTurnR.g < 3000)
		{
			hijoTurnR.h = DistanciaMH(hijoTurnR.st, destino);
			hijoTurnR.f = hijoTurnR.g + hijoTurnR.h;
			if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
				hijoTurnR.secuencia.push_back(actTURN_R);
				Abiertos.push(hijoTurnR);
			}
		}

	//	sleep(1);

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		checkEquipment(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna]);
		//cout << "nodo izquierda -> fila: " << hijoTurnL.st.fila << ", columna: " << hijoTurnL.st.columna << ". Bikini: " << hijoTurnL.bikini << ", zapatillas: " << hijoTurnL.zapatillas << endl;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		hijoTurnL.g += coste(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna], "t_left");
		if(hijoTurnL.g < 3000)
		{
			hijoTurnL.h = DistanciaMH(hijoTurnL.st, destino);
			hijoTurnL.f = hijoTurnL.g + hijoTurnL.h;
			if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
				hijoTurnL.secuencia.push_back(actTURN_L);
				Abiertos.push(hijoTurnL);
			}
		}

		//sleep(1);

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			checkEquipment(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna]);
			//cout << "nodo delante -> fila: " << hijoForward.st.fila << ", columna: " << hijoForward.st.columna << ". Bikini: " << hijoForward.bikini << ", zapatillas: " << hijoForward.zapatillas << endl;
			hijoForward.g += coste(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna], "forward");
			if(hijoForward.g < 3000)
			{
				hijoForward.h = DistanciaMH(hijoForward.st, destino);
				hijoForward.f = hijoForward.g + hijoForward.h;
				if (Cerrados.find(hijoForward.st) == Cerrados.end()){
					hijoForward.secuencia.push_back(actFORWARD);
					Abiertos.push(hijoForward);
				}
			}

		}

		//sleep(1);

		//Seleccionar el mejor nodo de ABIERTOS
		if (!Abiertos.empty()){
			current = Abiertos.top();
			//cout << "Siguiente nodo: "<< current.f << endl;
		}


	}

	cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;

}

int checkDest(nodo& current, const vector<estado> goals, int n_dest)
{
	if(current.actual_goal < n_dest)
	{
	//	cout << current.actual_goal << endl;
		if(current.st.fila == goals[current.actual_goal].fila and current.st.columna == goals[current.actual_goal].columna)
		{
			//cout << current.st.fila << " " <<  goals[current.actual_goal].fila << " "<<  current.st.columna << " " << goals[current.actual_goal].columna << endl;
			++current.actual_goal;
		}
	return current.actual_goal;
	}
	return 0;
}

bool ComportamientoJugador::busquedaPath(const estado &origen, const list<estado> &destinos, list<Action> &plan)
{
	
}

bool ComportamientoJugador::pathFinding_Astar_multi(const estado &origen, const list<estado> &destinos, list<Action> &plan)
{
	//Borro la lista
	cout << "Calculando plan A*\n";
	plan.clear();
	set<estado,ComparaEstados> Cerrados; 														// Lista de Cerrados
	priority_queue<nodo, vector<nodo>, open_comparison> Abiertos;		// Lista de Abiertos
	nodo mejor_padre;
	vector<nodo> hijos;
	nodo current;
	int actual_dest=0;
	//vector<estado> list_dest;
	this->destino = goals[actual_dest];
	current.actual_goal = actual_dest;
	current.st = origen;
	current.bikini = 0;
	current.zapatillas = 0;
	checkEquipment(current, mapaResultado[current.st.fila][current.st.columna]);
	current.g = coste(current, mapaResultado[current.st.fila][current.st.columna], "idle");
	//cout << "nodo actual -> fila: " << current.st.fila << ", columna: " << current.st.columna << ". Bikini: " << current.bikini << ", zapatillas: " << current.zapatillas << endl;
	current.h = DistanciaMH(current.st, goals[current.actual_goal]);
	current.f = current.g + current.h;
	current.secuencia.empty();
	current.dest_reached = 0;

	Abiertos.push(current);

	while (!Abiertos.empty() and (checkDest(current, goals, n_destinos) < 3)) // falta actualizar current.actual_goal
	{
		current.actual_goal = (current.actual_goal < 3) ? current.actual_goal : 0;
	//	cout << current.st.fila << " " << current.st.columna << endl;
		Abiertos.pop();
		Cerrados.insert(current.st);
	//	cout << "nodo actual -> fila: " << current.st.fila << ", columna: " << current.st.columna << endl;
		// se expande dicho nodo
		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		checkEquipment(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna]);
	//	cout << "nodo derecha -> fila: " << hijoTurnR.st.fila << ", columna: " << hijoTurnR.st.columna << ". Bikini: " << hijoTurnR.bikini << ", zapatillas: " << hijoTurnR.zapatillas << endl;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		//cout << hijoTurnR.g << endl;
		hijoTurnR.g += coste(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna], "t_right");
		//if(hijoTurnR.g < 3000)
		//{
			hijoTurnR.h = DistanciaMH(hijoTurnR.st, goals[current.actual_goal]);
			hijoTurnR.f = hijoTurnR.g + hijoTurnR.h;
			if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
				hijoTurnR.secuencia.push_back(actTURN_R);
				Abiertos.push(hijoTurnR);
			}
	//	}

	//	sleep(1);

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		checkEquipment(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna]);
	 ///cout << "nodo izquierda -> fila: " << hijoTurnL.st.fila << ", columna: " << hijoTurnL.st.columna << ". Bikini: " << hijoTurnL.bikini << ", zapatillas: " << hijoTurnL.zapatillas << endl;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		hijoTurnL.g += coste(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna], "t_left");
		//if(hijoTurnL.g < 3000)
		//{
			hijoTurnL.h = DistanciaMH(hijoTurnL.st, goals[current.actual_goal]);
			hijoTurnL.f = hijoTurnL.g + hijoTurnL.h;
			if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
				hijoTurnL.secuencia.push_back(actTURN_L);
				Abiertos.push(hijoTurnL);
			}
		//}

		//sleep(1);

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			checkEquipment(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna]);
			//cout << "nodo delante -> fila: " << hijoForward.st.fila << ", columna: " << hijoForward.st.columna << ". Bikini: " << hijoForward.bikini << ", zapatillas: " << hijoForward.zapatillas << endl;
			hijoForward.g += coste(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna], "forward");
			if(hijoForward.g < 3000)
			{
				hijoForward.h = DistanciaMH(hijoForward.st, goals[current.actual_goal]);
				hijoForward.f = hijoForward.g + hijoForward.h;
				if (Cerrados.find(hijoForward.st) == Cerrados.end()){
					hijoForward.secuencia.push_back(actFORWARD);
					Abiertos.push(hijoForward);
				}
			}

		}

		//sleep(1);

		//Seleccionar el mejor nodo de ABIERTOS
		if (!Abiertos.empty()){
			current = Abiertos.top();
			//cout << "Siguiente nodo: "<< current.f << endl;
		}
		cout << !Abiertos.empty() << " " << (checkDest(current, goals, n_destinos)<3) <<endl;
	}

	cout << "Terminada la busqueda\n";
	//cout << Abiertos.empty() << endl;
	cout << current.st.fila << " " <<  goals[current.actual_goal].fila << " "<<  current.st.columna << " " << goals[current.actual_goal].columna << endl;
	//cout << goals[current.actual_goal].fila << " " << goals[current.actual_goal].columna << endl;
	if (current.st.fila == goals[current.actual_goal].fila and current.st.columna == goals[current.actual_goal].columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;

}

// Sacar por la consola la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			cout << "A ";
		}
		else if (*it == actTURN_R){
			cout << "D ";
		}
		else if (*it == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}


// Funcion auxiliar para poner a 0 todas las casillas de una matriz
void AnularMatriz(vector<vector<unsigned char> > &m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}


// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			switch (cst.orientacion) {
				case 0: cst.fila--; break;
				case 1: cst.columna++; break;
				case 2: cst.fila++; break;
				case 3: cst.columna--; break;
			}
			mapaConPlan[cst.fila][cst.columna]=1;
		}
		else if (*it == actTURN_R){
			cst.orientacion = (cst.orientacion+1)%4;
		}
		else {
			cst.orientacion = (cst.orientacion+3)%4;
		}
		it++;
	}
}



int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}
