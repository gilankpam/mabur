// Paired A/B: same frames and same per-frame loss mask through the real codec under the frame-tail rule and R.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <thread>
#include <vector>
#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
using namespace mabur; using namespace std;
static const double OV[2]={0.5,0.25}, FP=0.010, TB=130e-6, TGAP=0.05; static const int BPB=4;
struct Res{long fr=0,both=0,onlyR=0,onlyD=0,onlyR_afterfail=0;};
static vector<char> decode(bool dyn,const vector<int>&ns,const vector<vector<char>>&srcM,const vector<vector<char>>&repM,int nfr){
  SwConfig c[2]={{64,64,OV[0]},{64,64,OV[1]}};
  SwEncoder E0(c[0],1000),E1(c[1],900000); SwEncoder*E[2]={&E0,&E1};
  SwDecoder D0(c[0],512),D1(c[1],512); SwDecoder*D[2]={&D0,&D1};
  vector<int> got(nfr,0); vector<double> done(nfr,-1),last(nfr,0); double air=0;
  for(int f=0;f<nfr;f++){int L=f&1,n=ns[f]; int w=dyn?frame_window(n,64):64; E[L]->begin_frame(n,frame_hold_syms(n,w,OV[L],BPB),dyn?w:0);
    vector<vector<uint8_t>> all; for(int i=0;i<n;i++){uint8_t p[62]={0};memcpy(p,&f,4);memcpy(p+4,&i,4);for(auto&e:E[L]->add_packet(p,62))all.push_back(move(e));}
    for(auto&e:E[L]->flush())all.push_back(move(e));
    vector<vector<uint8_t>> src,rep; for(auto&e:all){sw::SwHeader h;sw::parse_header(e.data(),e.size(),&h);(h.repair?rep:src).push_back(move(e));}
    air=max(air,f*FP);
    for(int part=0;part<2;part++){auto&v=part?rep:src;auto&m=part?repM[f]:srcM[f];
      for(size_t i=0,b=0;i<v.size();i+=BPB,b++){air+=TB; if(!part)last[f]=air; if(b<m.size()&&m[b])continue;
        for(size_t j=i;j<min(v.size(),i+BPB);j++){auto pk=D[L]->add_symbol(v[j].data(),v[j].size(),(uint64_t)(air*1000)+1);
          for(auto&p:pk){int g,k;memcpy(&g,p.data(),4);memcpy(&k,p.data()+4,4);if(g>=0&&g<nfr&&++got[g]==ns[g])done[g]=air;}}}}
  }
  vector<char> bad(nfr,0); for(int f=0;f<nfr;f++)bad[f]=got[f]<ns[f]||done[f]>last[f]+TGAP; return bad;
}
static void run(int kind,double eff,int burst,double nbar,uint64_t seed,int nfr,Res&o){
  mt19937_64 r(seed); lognormal_distribution<double> ln(log(nbar)-.5*.35*.35,.35); uniform_real_distribution<double> U(0,1);
  vector<int> ns; vector<vector<char>> sm(nfr),rm(nfr);
  double ml=kind==0?burst:.7+.3*8.5, ps=eff/(ml*(1-eff)+eff); uniform_int_distribution<int> lb(5,12); int left=0;
  auto draw=[&]{ if(left>0){left--;return (char)1;} if(U(r)<ps){int l=kind==0?burst:(U(r)<.7?1:lb(r)); left=l-1; return (char)1;} return (char)0;};
  for(int f=0;f<nfr;f++){int L=f&1,n=max(1,(int)lround(ln(r))); if(L==0&&f%200==0)n=min(310,n*3); ns.push_back(n);
    int sb=(n+BPB-1)/BPB, rb=((int)ceil(n*OV[L])+BPB-1)/BPB; sm[f].resize(sb); for(auto&c:sm[f])c=draw(); rm[f].resize(rb+1); for(int i=0;i<rb;i++)rm[f][i]=draw(); rm[f][rb]=0;}
  auto D=decode(false,ns,sm,rm,nfr), R=decode(true,ns,sm,rm,nfr);
  for(int f=8;f+8<nfr;f++){o.fr++; if(D[f]&&R[f])o.both++; else if(D[f])o.onlyD++; else if(R[f]){o.onlyR++; if(R[f-2]){o.onlyR_afterfail++;}}}
}
int main(int ac,char**av){int nfr=ac>1?atoi(av[1]):20000,seeds=ac>2?atoi(av[2]):6; struct J{int k;double e;int b;double n;int s;Res o;}; vector<J> jobs;
 for(double n:{30.,60.,88.,130.})for(auto sc:{make_tuple(0,.02,1),make_tuple(0,.05,3),make_tuple(1,.05,0),make_tuple(0,.10,3),make_tuple(1,.10,0)})for(int s=0;s<seeds;s++)jobs.push_back({get<0>(sc),get<1>(sc),get<2>(sc),n,s,{}});
 atomic_size_t idx{0};vector<thread> th;for(int t=0;t<24;t++)th.emplace_back([&]{size_t i;while((i=idx++)<jobs.size()){auto&j=jobs[i];run(j.k,j.e,j.b,j.n,1000*j.s+(uint64_t)j.n*7+(uint64_t)(j.e*1000)*3+j.b+j.k*91,nfr,j.o);}});for(auto&t:th)t.join();
 map<tuple<double,int,double,int>,Res> a;for(auto&j:jobs){auto&x=a[{j.n,j.k,j.e,j.b}];x.fr+=j.o.fr;x.both+=j.o.both;x.onlyR+=j.o.onlyR;x.onlyD+=j.o.onlyD;x.onlyR_afterfail+=j.o.onlyR_afterfail;}
 printf("n scen frames lost_both only_frame_tail only_R only_R_after_R_failed_prev  (%% of frames: both onlyD onlyR)\n");
 for(auto&kv:a){auto&[n,k,e,b]=kv.first;auto&x=kv.second;printf("%.0f %s%.0f%%b%d %ld %ld %ld %ld %ld (%.3f %.3f %.3f)\n",n,k?"mix":"fix",e*100,b,x.fr,x.both,x.onlyD,x.onlyR,x.onlyR_afterfail,100.*x.both/x.fr,100.*x.onlyD/x.fr,100.*x.onlyR/x.fr);}
}
