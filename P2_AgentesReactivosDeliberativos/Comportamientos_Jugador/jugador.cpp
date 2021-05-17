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
	goals.clear();
	for (int i=0; i<sensores.num_destinos; i++){
		estado aux;
		aux.fila = sensores.destino[2*i];
		aux.columna = sensores.destino[2*i+1];
		objetivos.push_back(aux);
		goals.push_back(aux);
	}
//	if(!hay_plan)
//		if(sensores.nivel==3)
	//		sortGoals(sensores);

	if(!hay_plan)
	{
		hay_plan = pathFinding (sensores.nivel, actual, objetivos, plan);
	}


	if (hay_plan && (sensores.destino[0] != destino.fila or sensores.destino[1] != destino.columna) && (sensores.nivel == 0 || sensores.nivel == 1  || sensores.nivel == 2))
	{
		cout << "El destino ha cambiado\n";
		hay_plan = false;
		//updateGoals(sensores);
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
			list<estado> objetivos_Astar = objetivos;
			//return pathFinding_Astar_multi(origen, objetivos_Astar, plan);
			return pathFinding_Astar(origen,objetivo_astar,plan);
			break;
		}
		case 3:
		{
			cout << "Optimo en coste 3 Objetivos - A*\n";
			list<estado> objetivos_Astar = objetivos;
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
	bool objetivo_alcanzado[3];
	int dest_reached;
};

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		return ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion) or
				(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.bikini < n.bikini) or
				(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.bikini == n.bikini and a.zapatillas < n.zapatillas));
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
				return (nodo.st.bikini) ? 10 : 200;
			case 'B':
				return (nodo.st.zapatillas) ? 15 : 100;
			case 'T':
				return 2;
			default:
				return 1;
		}
	}else if(accion == "t_right" || accion == "t_left")
	{
		switch (terreno) {
			case 'A':
				return (nodo.st.bikini) ? 5 : 500;
			case 'B':
				return (nodo.st.zapatillas) ? 1 : 3;
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
	 	nodo.st.bikini = true;
	else if (celda == 'D')
		nodo.st.zapatillas = true;
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
	current.st.bikini = 0;
	current.st.zapatillas = 0;
	checkEquipment(current, mapaResultado[current.st.fila][current.st.columna]);
	current.g = coste(current, mapaResultado[current.st.fila][current.st.columna], "idle");
	current.h = DistanciaMH(current.st, destino);
	current.f = current.g + current.h;
	current.secuencia.empty();

	Abiertos.push(current);

	while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)) // falta actualizar current.actual_goal
	{
		Abiertos.pop();
		Cerrados.insert(current.st);

		// se expande dicho nodo

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		checkEquipment(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna]);
	//cout << "nodo derecha -> fila: " << hijoTurnR.st.fila << ", columna: " << hijoTurnR.st.columna << ". Bikini: " << hijoTurnR.bikini << ", zapatillas: " << hijoTurnR.zapatillas << endl;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
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

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		checkEquipment(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna]);
	 ///cout << "nodo izquierda -> fila: " << hijoTurnL.st.fila << ", columna: " << hijoTurnL.st.columna << ". Bikini: " << hijoTurnL.bikini << ", zapatillas: " << hijoTurnL.zapatillas << endl;
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

		//Seleccionar el mejor nodo de ABIERTOS
		if (!Abiertos.empty()){
			current = Abiertos.top();
			//cout << "Siguiente nodo: "<< current.f << endl;
		}
	}

	cout << "Terminada la busqueda\n";
	if(Abiertos.empty()) cout << "Lista de abiertos vacia" << endl;
	if(current.st.fila==destino.fila and current.st.columna == destino.columna)
	{
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

void resetLists(nodo &current, set<estado,ComparaEstados> &cerrados, priority_queue<nodo, vector<nodo>, open_comparison> &abiertos)
{
	cerrados.clear();
	priority_queue<nodo, vector<nodo>, open_comparison> aux;
	aux.push(current);
	abiertos.swap(aux);
}

int checkDest(nodo& current, const vector<estado> goals, int n_dest, set<estado,ComparaEstados> &cerrados, priority_queue<nodo, vector<nodo>, open_comparison> &abiertos)
{
	if(current.dest_reached < n_dest)
	{
		for(int i=0; i<n_dest; ++i)
		{
			if(current.objetivo_alcanzado[i] == false)
				if(current.st.fila == goals[i].fila and current.st.columna == goals[i].columna)
				{
					++current.dest_reached;
					current.objetivo_alcanzado[i] = true;
					cout << "--- destino alcanzado: " << i << ", fila: " << goals[i].fila << ", columna: " << goals[i].columna << " ---" <<endl;
					cout << "destinos encontrados: " << current.dest_reached << endl;
					resetLists(current, cerrados, abiertos);
					break;
				}
		}

	return current.dest_reached;
	}
	return 0;
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
	this->destino = goals[actual_dest];
	current.st = origen;
	current.st.bikini = 0;
	current.st.zapatillas = 0;
	checkEquipment(current, mapaResultado[current.st.fila][current.st.columna]);
	current.g = coste(current, mapaResultado[current.st.fila][current.st.columna], "idle");
	for(int i=0; i<n_destinos; ++i)
	{
		current.h += DistanciaMH(current.st, goals[i]);
		current.objetivo_alcanzado[i] = false;
	}
	current.f = current.g + current.h;
	current.secuencia.empty();
	current.dest_reached = 0;

	Abiertos.push(current);

	while (!Abiertos.empty() and (checkDest(current, goals, n_destinos, Cerrados, Abiertos) < n_destinos)) // falta actualizar current.actual_goal
	{
		Abiertos.pop();
		Cerrados.insert(current.st);
		// se expande dicho nodo
		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		checkEquipment(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna]);
	//cout << "nodo derecha -> fila: " << hijoTurnR.st.fila << ", columna: " << hijoTurnR.st.columna << ". Bikini: " << hijoTurnR.bikini << ", zapatillas: " << hijoTurnR.zapatillas << endl;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		hijoTurnR.g += coste(hijoTurnR, mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna], "t_right");
		if(hijoTurnR.g < 3000)
		{
			hijoTurnR.h=0;
			for(int i=0; i<n_destinos; ++i)
				if(!hijoTurnR.objetivo_alcanzado[i])
					hijoTurnR.h += DistanciaMH(hijoTurnR.st, goals[i]);
			hijoTurnR.f = hijoTurnR.g + hijoTurnR.h;
			if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
				hijoTurnR.secuencia.push_back(actTURN_R);
				Abiertos.push(hijoTurnR);
			}
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		checkEquipment(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna]);
	 ///cout << "nodo izquierda -> fila: " << hijoTurnL.st.fila << ", columna: " << hijoTurnL.st.columna << ". Bikini: " << hijoTurnL.bikini << ", zapatillas: " << hijoTurnL.zapatillas << endl;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		hijoTurnL.g += coste(hijoTurnL, mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna], "t_left");
		if(hijoTurnL.g < 3000)
		{
			hijoTurnL.h=0;
			for(int i=0; i<n_destinos; ++i)
				if(!hijoTurnL.objetivo_alcanzado[i])
					hijoTurnL.h += DistanciaMH(hijoTurnL.st, goals[i]);
			hijoTurnL.f = hijoTurnL.g + hijoTurnL.h;
			if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
				hijoTurnL.secuencia.push_back(actTURN_L);
				Abiertos.push(hijoTurnL);
			}
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			checkEquipment(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna]);
			//cout << "nodo delante -> fila: " << hijoForward.st.fila << ", columna: " << hijoForward.st.columna << ". Bikini: " << hijoForward.bikini << ", zapatillas: " << hijoForward.zapatillas << endl;
			hijoForward.g += coste(hijoForward, mapaResultado[hijoForward.st.fila][hijoForward.st.columna], "forward");
			if(hijoForward.g < 3000)
			{
				hijoForward.h=0;
				for(int i=0; i<n_destinos; ++i)
					if(!hijoForward.objetivo_alcanzado[i])
						hijoForward.h += DistanciaMH(hijoForward.st, goals[i]);
				hijoForward.f = hijoForward.g + hijoForward.h;
				if (Cerrados.find(hijoForward.st) == Cerrados.end()){
					hijoForward.secuencia.push_back(actFORWARD);
					Abiertos.push(hijoForward);
				}
			}

		}


		//Seleccionar el mejor nodo de ABIERTOS
		if (!Abiertos.empty()){
			current = Abiertos.top();
			//cout << "Siguiente nodo: "<< current.f << endl;
		}
	}

	cout << "Terminada la busqueda\n";
	if(Abiertos.empty()) cout << "Lista de abiertos vacia" << endl;
	if(current.dest_reached == n_destinos)
	{
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
