// A/B on the real SwEncoder/SwDecoder: frame-tail rule vs frame_window. Same frame and loss model as fecsim.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <thread>
#include <atomic>
#include <vector>
#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
using namespace mabur; using namespace std;
static const double OV[2]={0.5,0.25}, FP=0.010, TB=130e-6, TGAP=0.05; static const int BPB=4;
struct Env{int L; vector<uint8_t> b;}; struct Body{vector<Env> e; double ready; double t; bool lost=false;};
struct R{double fr=0,tr=0,late=0;};
static void run(bool dyn,int kind,double eff,int burst,double nbar,uint64_t seed,int nfr,R&o){
  mt19937_64 r(seed); lognormal_distribution<double> ln(log(nbar)-.5*.35*.35,.35); uniform_real_distribution<double> U(0,1);
  SwConfig c[2]={{64,64,OV[0]},{64,64,OV[1]}};
  SwEncoder E0(c[0],1000),E1(c[1],900000); SwEncoder* E[2]={&E0,&E1};
  SwDecoder D0(c[0],512),D1(c[1],512); SwDecoder* D[2]={&D0,&D1};
  vector<Body> bodies; vector<int> ns; bodies.reserve(nfr*30);
  for(int f=0;f<nfr;f++){int L=f&1; double t0=f*FP; int n=max(1,(int)lround(ln(r))); if(L==0&&f%200==0)n=min(310,n*3); ns.push_back(n);
    int w=dyn?frame_window(n,64):64; E[L]->begin_frame(n,frame_hold_syms(n,w,OV[L],BPB),dyn?w:0);
    vector<vector<uint8_t>> all; for(int i=0;i<n;i++){uint8_t p[62]={0}; memcpy(p,&f,4); memcpy(p+4,&i,4); for(auto&e:E[L]->add_packet(p,62))all.push_back(move(e));}
    for(auto&e:E[L]->flush())all.push_back(move(e));
    vector<Env> src,rep; for(auto&e:all){sw::SwHeader h; sw::parse_header(e.data(),e.size(),&h); (h.repair?rep:src).push_back({L,move(e)});}
    for(int part=0;part<2;part++){auto&v=part?rep:src; for(size_t i=0;i<v.size();i+=BPB){Body b; b.ready=t0; for(size_t j=i;j<min(v.size(),i+BPB);j++)b.e.push_back(move(v[j])); bodies.push_back(move(b));}}
  }
  double air=0; for(auto&b:bodies){air=max(air,b.ready);b.t=air+TB;air=b.t;}
  double meanlen=kind==0?burst:.7+.3*8.5, ps=eff/(meanlen*(1-eff)+eff); uniform_int_distribution<int> lb(5,12);
  for(size_t i=0;i<bodies.size();){ if(U(r)<ps){int l=kind==0?burst:(U(r)<.7?1:lb(r)); for(int j=0;j<l&&i<bodies.size();j++,i++)bodies[i].lost=true;} else i++; }
  vector<int> got(nfr,0); vector<double> done(nfr,-1), send(nfr,0);
  // nominal end of a frame's sources: its first bodies ready at f*FP, counted from the schedule.
  for(auto&b:bodies){ sw::SwHeader h; if(b.e.empty())continue; sw::parse_header(b.e[0].b.data(),b.e[0].b.size(),&h); if(!h.repair){int f=(int)lround(b.ready/FP); send[f]=max(send[f],b.t);} }
  for(auto&b:bodies){ if(b.lost)continue; for(auto&e:b.e){ auto pk=D[e.L]->add_symbol(e.b.data(),e.b.size(),(uint64_t)(b.t*1000)+1); for(auto&p:pk){int f,i; memcpy(&f,p.data(),4); memcpy(&i,p.data()+4,4); if(f>=0&&f<nfr&&++got[f]==ns[f])done[f]=b.t;} } }
  for(int f=8;f+8<nfr;f++){o.fr++; bool tr=got[f]<ns[f]||done[f]>send[f]+TGAP; if(tr)o.tr++; if(got[f]<ns[f]||done[f]-send[f]>.010)o.late++;}
}
int main(int ac,char**av){int nfr=ac>1?atoi(av[1]):20000,seeds=ac>2?atoi(av[2]):6;
 struct J{bool d;int k;double e;int b;double n;int s;R o;}; vector<J> jobs;
 for(double n:{30.,60.,88.,130.})for(auto sc:{make_tuple(0,.02,1),make_tuple(0,.05,3),make_tuple(1,.05,0)})for(bool d:{false,true})for(int s=0;s<seeds;s++)jobs.push_back({d,get<0>(sc),get<1>(sc),get<2>(sc),n,s,{}});
 atomic_size_t idx{0}; vector<thread> th; for(int t=0;t<24;t++)th.emplace_back([&]{size_t i;while((i=idx++)<jobs.size()){auto&j=jobs[i];run(j.d,j.k,j.e,j.b,j.n,1000*j.s+(uint64_t)j.n*7+(uint64_t)(j.e*1000)*3+j.b,nfr,j.o);}}); for(auto&t:th)t.join();
 map<tuple<double,int,double,int,bool>,R> a; for(auto&j:jobs){auto&x=a[{j.n,j.k,j.e,j.b,j.d}];x.fr+=j.o.fr;x.tr+=j.o.tr;x.late+=j.o.late;}
 printf("n kind eff burst rule trunc%% late10%%\n"); for(auto&kv:a){auto&[n,k,e,b,d]=kv.first;printf("%.0f %d %.2f %d %s %.3f %.3f\n",n,k,e,b,d?"R":"D",100*kv.second.tr/kv.second.fr,100*kv.second.late/kv.second.fr);}
}
