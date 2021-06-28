#include <iostream>
#include <cstdlib>
#include <vector>
#include <queue>
#include "player.h"
#include "environment.h"

using namespace std;

const double masinf=9999999999.0, menosinf=-9999999999.0;
const int n_rows=7, n_cols=7;


// Constructor
Player::Player(int jug){
    jugador_=jug;
}

// Actualiza el estado del juego para el jugador
void Player::Perceive(const Environment & env){
    actual_=env;
}

double Puntuacion(int jugador, const Environment &estado){
    double suma=0;

    for (int i=0; i<7; i++)
      for (int j=0; j<7; j++){
         if (estado.See_Casilla(i,j)==jugador){
            if (j<3)
               suma += j;
            else
               suma += (6-j);
         }
      }

    return suma;
}


// Funcion de valoracion para testear Poda Alfabeta
double ValoracionTest(const Environment &estado, int jugador){
    int ganador = estado.RevisarTablero();

    if (ganador==jugador)
       return 99999999.0; // Gana el jugador que pide la valoracion
    else if (ganador!=0)
            return -99999999.0; // Pierde el jugador que pide la valoracion
    else if (estado.Get_Casillas_Libres()==0)
            return 0;  // Hay un empate global y se ha rellenado completamente el tablero
    else
          return Puntuacion(jugador,estado);
}

// ------------------- Los tres metodos anteriores no se pueden modificar



int getNearby(int n_squares, int player, const Environment &state)
{
  int nearby_v = 1, nearby_h = 1, nearby_dn=1, nearby_dp=1;
  int n_v=0, n_h=0, n_d=0;
  for(int i=0; i<n_rows; ++i)
    for(int j=0; j<n_cols; ++j)
    {
      if(state.See_Casilla(i,j) == player) // 0 vacia, 1 jugador1, 2 jugador2.
      {
        int i_index=i-1, j_index=j;
      //  bool continue=true;
        while((i_index>=0)) // vertical, on       metodo nuevo: recorre la columna hasta que encuentra una casilla con player, si es player cuenta a ver cuantas seguidas hay de ese player. devuelve el numero de fichas seguidas y dentro del for se hace ++ al numero de filas que ha devuelto
        {
          if(state.See_Casilla(i_index,j_index) == player)
          {
            if(nearby_v!=n_squares)
            {
              ++nearby_v;
            }else
            {
              ++n_v;
              nearby_v=0;
            }
          }
          else
            break;
          --i_index;
        }
        nearby_v=0;

        i_index = i;
        j_index = j-1;
        while((j_index>=0)) // horizontal, left
        {
          if(state.See_Casilla(i_index,j_index) == player)
          {
            if(nearby_h!=n_squares)
            {
              ++nearby_h;
            }else
            {
              ++n_h;
              nearby_h=0;
            }
          }
          else
            break;
          --j_index;
        }



        i_index = i-1;
        j_index = j-1;
        while((j_index>=0 && i_index>=0)) // diagonal, negative slope(leftside)
        {
          if(state.See_Casilla(i_index,j_index) == player)
          {
            if(nearby_dn!=n_squares)
            {
              ++nearby_dn;
            }else
            {
              ++n_d;
              nearby_dn=0;
            }
          }
          else
            break;
          --i_index;--j_index;
        }


        i_index = i+1;
        j_index = j-1;
        while((j_index>=0 && i_index<n_rows)) // diagonal, positive slope(leftside)
        {
          if(state.See_Casilla(i_index,j_index) == player)
          {
            if(nearby_dp!=n_squares)
            {
              ++nearby_dp;
            }else
            {
              ++n_d;
              nearby_dp=0;
            }
          }
          else
            break;
          ++i_index;--j_index;
        }

      }
    }
  return n_h+n_v+n_d;
}


/* Función heuristica
 Características: número de filas, columnas y diagonales adyacentes(2, 3 o 4)
 Estados finales: tableros completos o con línea ganadora
 Estados ganadores para un jugador: estados finales en los que no le toca poner
 Primer enfoque: usar combinación lineal usando pesos ponderados que determinan la importancia de cada característica
*/
double heuristic(int player, const Environment &state){
  int opponent = (player == 1) ? 2 : 1;
  int features[3], opponent_features[3];
  for(int i=0; i<3; ++i)
  {
    features[2-i] = getNearby(4-i, player, state);
    opponent_features[2-i] = getNearby(4-i, opponent, state);
  }

  int player_h   = features[0]*10+features[1]*100+features[2]*10000,
      opponent_h = opponent_features[0]*10+opponent_features[1]*100+opponent_features[2]*1000;
  return (player_h - opponent_h);
}

double Valoracion(const Environment &estado, int jugador){
  int ganador = estado.RevisarTablero();
  //cout << "valoracion para el jugador " <<jugador;
  if (ganador==jugador)
    return 9999999999.0; // Gana el jugador que pide la valoracion
  else if (ganador!=0)
    return -9999999999.0; // Pierde el jugador que pide la valoracion
  else if (estado.Get_Casillas_Libres()==0)
    return 0;  // Hay un empate global y se ha rellenado completamente el tablero
  else
  {
    double h = heuristic(jugador,estado);
    //cout << " :" << h << endl;
    return h;
  }

}

// Esta funcion no se puede usar en la version entregable
// Aparece aqui solo para ILUSTRAR el comportamiento del juego
// ESTO NO IMPLEMENTA NI MINIMAX, NI PODA ALFABETA
void JuegoAleatorio(bool aplicables[], int opciones[], int &j){
    j=0;
    for (int i=0; i<8; i++){
        if (aplicables[i]){
           opciones[j]=i;
           j++;
        }
    }
}

// Invoca el siguiente movimiento del jugador
Environment::ActionType Player::Think(){
    const int PROFUNDIDAD_MINIMAX = 6;  // Umbral maximo de profundidad para el metodo MiniMax
    const int PROFUNDIDAD_ALFABETA = 8; // Umbral maximo de profundidad para la poda Alfa_Beta

    Environment::ActionType accion; // acci�n que se va a devolver
    bool aplicables[8]; // Vector bool usado para obtener las acciones que son aplicables en el estado actual. La interpretacion es
                        // aplicables[0]==true si PUT1 es aplicable
                        // aplicables[1]==true si PUT2 es aplicable
                        // aplicables[2]==true si PUT3 es aplicable
                        // aplicables[3]==true si PUT4 es aplicable
                        // aplicables[4]==true si PUT5 es aplicable
                        // aplicables[5]==true si PUT6 es aplicable
                        // aplicables[6]==true si PUT7 es aplicable
                        // aplicables[7]==true si BOOM es aplicable



    double valor; // Almacena el valor con el que se etiqueta el estado tras el proceso de busqueda.
    double alpha, beta; // Cotas de la poda AlfaBeta

    int n_act; //Acciones posibles en el estado actual


    n_act = actual_.possible_actions(aplicables); // Obtengo las acciones aplicables al estado actual en "aplicables"
    int opciones[10];

    // Muestra por la consola las acciones aplicable para el jugador activo
    //actual_.PintaTablero();
    cout << " Acciones aplicables ";
    (jugador_==1) ? cout << "Verde: " : cout << "Azul: ";
    for (int t=0; t<8; t++)
      if (aplicables[t])
         cout << " " << actual_.ActionStr( static_cast< Environment::ActionType > (t)  );
    cout << endl;

    int maximizingPlayer = (actual_.Last_Action(1) == -1 && actual_.Last_Action(2) == -1) ? 1: 0; // si es el primer turno comienza con un unico turno
    valor = AlphaBeta_Pruning(actual_, jugador_, PROFUNDIDAD_ALFABETA, accion, alpha, beta, maximizingPlayer);
    cout << "Valor AlphaBeta: " << valor << "  Accion: " << actual_.ActionStr(accion) << endl;

    return accion;
}

bool terminalState(const Environment& env, int depth)
{
  bool options[8];
  int n_options = env.possible_actions(options);
  //cout << "depth: " << depth << ", jterminado: " << env.JuegoTerminado() << ", nopciones: " << n_options << endl;
  return (depth == 0 || env.JuegoTerminado() || n_options==0) ? true : false;
}

double Player::maxValue(const Environment& env, int player, int depth, double alpha, double beta, Environment::ActionType& action, int maximizingPlayer, int next_move)
{
  if(terminalState(env, depth))
  {
    // cout <<"termina" <<endl;
    //action = static_cast<Environment::ActionType> (next_move);
    return Valoracion(env, player);
  }

  bool options[8];
  double value = menosinf, max_value;
  int n_options = env.possible_actions(options),
      rival = (player==1) ? 2 : 1;
  next_move=-1;
  ++maximizingPlayer;
  //cout << "maximizingPlayer " << maximizingPlayer << " en max, opciones posibles: "<<n_options <<endl;
  for(int i=0; i<n_options; ++i)
  {
    Environment child = env.GenerateNextMove(next_move);
    //action = static_cast<Environment::ActionType> (next_move);
    if(player == child.JugadorActivo()) // mirar que el primer turno sea solo 1 ficha y en el resto 2
      max_value = maxValue(child, player, depth-1, alpha, beta, action, maximizingPlayer, next_move);
    else
      max_value = minValue(child, rival, depth-1, alpha, beta, action, maximizingPlayer, next_move);

    if(max_value > value)
    {
      value = max_value;
      action = static_cast<Environment::ActionType> (next_move);
    }

    if(beta <= value) return value;  // beta cutoff

    alpha = max(alpha, value);
  }
  return value;
}

double Player::minValue(const Environment& env, int player, int depth, double alpha, double beta, Environment::ActionType& action, int maximizingPlayer, int next_move)
{
  if(terminalState(env, depth))
  {
    return Valoracion(env, player);
  }

  bool options[8];
  double value = masinf, min_value;
  int n_options = env.possible_actions(options),
      rival = (player==1) ? 2 : 1;
      next_move=-1,
      min_value;
  ++maximizingPlayer;
  for(int i=0; i<n_options; ++i)
  {
    Environment child = env.GenerateNextMove(next_move);
    //action = static_cast<Environment::ActionType> (next_move);
    if(player == child.JugadorActivo())
      min_value = minValue(child, player, depth-1, alpha, beta, action, maximizingPlayer, next_move);
    else
      min_value = maxValue(child, rival, depth-1, alpha, beta, action, 0, next_move);

    if(min_value < value)
    {
      value = min_value;
      action = static_cast<Environment::ActionType> (next_move);
    }

    if(value <= alpha)
    {
      return value;  // alpha cutoff
    }
    beta = min(beta, value);
  }
  return value;
}

// metodo a tener en cuenta para comprobar si el tablero se llena(de cara a estado final) -> Get_Ocupacion_Columna
// The Alpha-Beta algorithm, from Russell and Norvig's AI textbook (Russell and Norvig 2016).
// https://www.researchgate.net/figure/The-Alpha-Beta-pseudo-code-from-Russell-and-Norvigs-AI-textbook-Russell-and-Norvig_fig1_329715244
double Player::AlphaBeta_Pruning(const Environment& env, int player, int depth, Environment::ActionType& action, double alpha, double beta, int maximizingPlayer)
{
  double value;
  bool options[8];
  int rival = (player==1) ? 2 : 1;
  int next_move = -1;

  value = maxValue(env, player, depth-1, menosinf, masinf, action, maximizingPlayer, next_move);
  return value;
}
