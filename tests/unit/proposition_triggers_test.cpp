#include "proposition_triggers.hpp"
#include <stdexcept>
using harness::tiny::PropositionTriggers;
void check(bool passed) { if (!passed) throw std::runtime_error("proposition trigger regression"); }
int main() {
  PropositionTriggers gate;
  gate.update("grasp",true,true,0);gate.update("human",true,true,0);
  check(gate.next(0)=="grasp");check(gate.next(0)=="human");check(!gate.next(0));
  gate.update("grasp",false,false,1);gate.update("grasp",true,true,2);check(!gate.next(2));
  gate.update("grasp",true,false,3);gate.update("grasp",true,true,4);check(!gate.next(4));
  gate.update("grasp",true,true,10000000001LL);check(gate.next(10000000001LL)=="grasp");
  gate.reset();gate.update("grasp",true,true,0);check(!gate.next(250000001));
  gate.reset();gate.update("human",true,true,0);gate.update("human",true,false,1);check(!gate.next(1));
}
