#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <string>

struct estado {
  int fila;
  int columna;
  int orientacion;
  bool bikini;
  bool zapatillas;
  bool operator==(const estado& e)
  {
    return (e.fila == fila && e.columna == columna);
  }
};

enum class nivel4_state {start, approach, plannin, blockin, done};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      hay_plan = false;
      hay_plan = goal_reached = goal_spotted = false;
      n_swept = 0;
      state = nivel4_state::start;
      last_action = actIDLE;
      current_cost = 0;
      same_block = 0;
      try_again = false;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      hay_plan = false;
      state = nivel4_state::plannin;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    estado actual;
    list<estado> objetivos;
    estado destino;
    list<Action> plan;
    bool hay_plan;
    bool bikini;
    bool zapatillas;
    int nivel_bateria;
    int n_destinos;
    //vector<pair<int,int>> goals;
    vector<estado> goals;
    bool goal_reached;
    bool goal_spotted;
    nivel4_state state;
    int n_swept;
    Action last_action;
    int current_cost;
    int n_destinos_a_encontrar;
    int same_block;
    estado last_block;
    estado destino_encontrado;
    bool try_again;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const list<estado> &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Astar(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Astar_multi(const estado &origen, const list<estado> &destinos, list<Action> &plan);

    int DistanciaMH(const estado& x, const estado& y);
    int getCosteMovimiento(estado &origen, estado &sig);
    void updateMapa(vector<vector<unsigned char>> &mapa, Sensores sensores);
    Action swept(Sensores sensores);
    Action playerApproach();
    bool destinoAlcanzado();

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);

};

#endif
