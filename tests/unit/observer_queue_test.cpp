#include "observer_queue.hpp"
#include "evidence.hpp"
#include "execution_sampling.hpp"
#include <future>
#include <iostream>
#include <thread>
using namespace harness::observation;
namespace {
void stalled_consumer() {
  ObserverQueue queue;
  const Json config={{"topics",Json::array({{{"name","/camera"},{"role","camera"}},
                                           {{"name","/joints"},{"role","joint_state"}}})}};
  Evidence evidence(config,"queue-test");
  const Ns start=1000000000LL,end=start+15000000000LL;
  evidence.tick(start);
  // Processing is blocked while another thread continues receiving an entire
  // synthetic 15s window. Processing time must not become acquisition time.
  auto producer=std::async(std::launch::async,[&]{
    for(Ns t=start;t<=end;t+=50000000LL)
      for(const auto *topic:{"/camera","/joints"})
        queue.push(1024,[t]{return t;},[&,topic,t]{
          evidence.tick(t);evidence.sample(topic,t,t,monotonic_ns(),true);
        });
  });
  producer.get();
  require(queue.status()["queued_messages"]==602,"producer blocked or lost packets while consumer stalled");
  Ns previous=-1;
  for(int i=0;i<602;++i) {
    auto task=queue.pop();require(task.has_value(),"missing queued packet");
    require(task->ros>=previous,"receive timestamp order changed");previous=task->ros;task->run();
  }
  evidence.tick(end);
  auto &window=evidence.capture(15.,0.,"unlabeled");evidence.tick(end);
  require(sample_coverage(config,window.at("coverage"),start,end),"processing pause became false recording gap");
  for(const auto &c:window.at("coverage"))
    require(c.at("max_receipt_gap_ns")==50000000LL && c.at("count")==301,
            "receive timestamps were replaced by processing timestamps");
  require(queue.status()["queued_bytes"]==0,"consumed packets retain queue byte budget");
}
void overflow() {
  for(bool byte_limit:{true,false}) {
    ObserverQueue queue(byte_limit?3:100,byte_limit?100:1);
    queue.push(3,[]{return 1;},[]{});
    queue.push(1,[]{return 2;},[]{});
    require(queue.status()["overflow"]==true,"queue failed to enforce capacity");
    bool failed=false;try{queue.pop();}catch(const std::exception &){failed=true;}
    require(failed,"overflow silently continued incomplete recording");
  }
}
void periodic() {
  ObserverQueue queue;
  queue.push(0,[]{return 1;},[]{},"critic");
  for(int i=0;i<1000;++i)queue.push(0,[]{return 2;},[]{},"critic");
  queue.push(1,[]{return 3;},[]{});
  require(queue.status()["queued_messages"]==2,"periodic work grows without bound");
  require(queue.pop()->ros==1 && queue.pop()->ros==3,"coalescing reordered data");
  queue.push(0,[]{return 4;},[]{},"critic");
  require(queue.pop()->ros==4,"periodic work never resumes");
}
void concurrent() {
  ObserverQueue queue;Ns clock=0;
  auto produce=[&]{for(int i=0;i<200;++i)queue.push(1,[&]{return ++clock;},[]{});};
  std::jthread a(produce),b(produce);a.join();b.join();
  for(Ns i=1;i<=400;++i)require(queue.pop()->ros==i,"concurrent clock capture reordered messages");
}
void reset() {
  ObserverQueue queue;
  queue.push(1,[]{return 100;},[]{});queue.push(1,[]{return 1;},[]{});
  require(queue.pop()->ros==100 && queue.pop()->ros==1,"ROS clock reset was hidden");
  require(!queue.pop(),"idle consumer should yield");
}
}
int main(){try{stalled_consumer();overflow();periodic();concurrent();reset();
  std::cout<<"5 observer receive queue check groups passed\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
