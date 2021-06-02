#ifndef PLAYER_H
#define PLAYER_H

#include "environment.h"

class Player{
    public:
      Player(int jug);
      Environment::ActionType Think();
      void Perceive(const Environment &env);
      double AlphaBeta_Pruning(const Environment &env, int player, int depth, Environment::ActionType& action, double alpha, double beta, bool maximizingPlayer);
    private:
      int jugador_;
      Environment actual_;


};
#endif
