#ifndef PLAYER_H
#define PLAYER_H

#include "environment.h"

class Player{
    public:
      Player(int jug);
      Environment::ActionType Think();
      void Perceive(const Environment &env);
      double maxValue(const Environment& env, int player, int depth, double alpha, double beta, Environment::ActionType& action, int maximizingPlayer, int next_move);
      double minValue(const Environment& env, int player, int depth, double alpha, double beta, Environment::ActionType& action, int maximizingPlayer, int next_move);
      double AlphaBeta_Pruning(const Environment &env, int player, int depth, Environment::ActionType& action, double alpha, double beta, int maximizingPlayer);
    private:
      int jugador_;
      Environment actual_;


};
#endif
