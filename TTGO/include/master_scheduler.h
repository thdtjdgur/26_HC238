#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Legacy IDs remain on air. A1/A2/A3 and FF F2 are exclusively UGV traffic.
namespace master {
constexpr uint8_t GCS=0xFF, UGV=0xFE, UAV=0xFD, ENVELOPE=0xA4;
constexpr uint8_t POLL=1, WAYPOINT=2, ACK=3, UAV_GPS=0x10, UAV_DETECTION=0x11;
constexpr uint32_t PERIOD=3000, GUARD=8, RESPONSE=180, UAV_RESPONSE=300, FRESH=3200;
constexpr size_t HEADER=7;
inline bool due(uint32_t now, uint32_t deadline) { return int32_t(now-deadline)>=0; }
inline uint16_t sequence(const uint8_t *p) { return uint16_t(p[4]) | uint16_t(p[5])<<8; }
inline bool envelope(const uint8_t *p, size_t n) {
    return n>=HEADER && n<=128 && p[0]==ENVELOPE && n==HEADER+p[6];
}
inline void header(uint8_t *p,uint8_t src,uint8_t dst,uint8_t type,uint16_t seq,uint8_t len) {
    p[0]=ENVELOPE; p[1]=src; p[2]=dst; p[3]=type;
    p[4]=uint8_t(seq); p[5]=uint8_t(seq>>8); p[6]=len;
}
// SF7/BW125/CR4/5, explicit header, CRC on. Preamble is read from the radio.
inline uint32_t airtime(uint16_t bytes, uint16_t preamble=8) {
    int32_t numerator=8*bytes-4*7+28+16;
    uint32_t symbols=8+(numerator>0 ? uint32_t((numerator+27)/28)*5 : 0);
    return ((uint32_t(preamble)+symbols)*1024+4352+999)/1000;
}
struct Frame { uint8_t data[1029]; uint16_t size=0; uint32_t at=0; };
struct Command {
    uint8_t data[128]; uint8_t size=0, attempts=0; uint32_t cycle=0;
};
struct IO {
    void (*start)(const uint8_t *,uint8_t);
    void (*receive)();
    void (*abort)();
    void (*event)(const char *,uint8_t,uint16_t);
};
enum class State { IDLE, UGV_RTCM_TX, UGV_GRANT_TX, WAIT_UGV,
                   UAV_REQUEST_TX, WAIT_UAV, OPTIONAL_TX, WAIT_ACK };

class Scheduler {
public:
    explicit Scheduler(IO io): io_(io) {}
    State state() const { return state_; }
    bool transmitting() const { return busy_; }
    bool enabled() const { return enabled_; }
    void set_preamble(uint16_t n) { preamble_=n; }
    void stage(const uint8_t *p,uint16_t n,uint16_t type,uint32_t now) {
        int slot=type==1004?0:type==1006?1:type==1012?2:type==1230?3:-1;
        if(slot<0 || n>1029) return;
        memcpy(staging_[slot].data,p,n); staging_[slot].size=n; staging_[slot].at=now;
    }
    void commit(uint32_t now) {
        for(int i=0;i<4;++i) { ready_[i]=staging_[i]; staging_[i].size=0; }
        if(!enabled_) { enabled_=true; next_=now+200; }
    }
    void discard_staging() { for(auto &f:staging_) f.size=0; }
    bool command(const uint8_t *p,uint8_t n) {
        if(!envelope(p,n) || p[1]!=GCS || (p[2]!=UGV && p[2]!=UAV) || p[3]!=WAYPOINT) return false;
        if(p[6]<9 || p[7]==0 || p[7]>15 || p[6]!=1+8*p[7]) return false;
        for(uint8_t i=0;i<p[7];++i) {
            int32_t lat,lon; memcpy(&lat,p+8+8*i,4); memcpy(&lon,p+12+8*i,4);
            if(lat < -900000000 || lat > 900000000 || lon < -1800000000 || lon > 1800000000) return false;
        }
        int slot=p[2]==UGV?0:1;
        if(commands_[slot].size) io_.event("SUPERSEDED",p[2],sequence(commands_[slot].data));
        memcpy(commands_[slot].data,p,n); commands_[slot].size=n;
        commands_[slot].attempts=0; commands_[slot].cycle=0;
        io_.event("QUEUED",p[2],sequence(p)); return true;
    }
    // Called only for CRC-clean packets after restoring RX, before logging.
    bool received(const uint8_t *p,uint8_t n,uint32_t now) {
        if(busy_ || due(now,wait_until_)) return false;
        if(state_==State::WAIT_UGV && (n==14 || n==23) &&
           p[0]==UGV && p[2]==0xF3 && p[3]==1) {
            io_.event("UGV_RX",UGV,p[1]); state_=State::UAV_REQUEST_TX; guard_until_=now+GUARD; return true;
        }
        if(!envelope(p,n) || p[1]!=UAV || p[2]!=GCS) return false;
        if(state_==State::WAIT_UAV && sequence(p)==poll_seq_ &&
           ((p[3]==UAV_GPS && (p[6]==8 || p[6]==9)) ||
            (p[3]==UAV_DETECTION && p[6]==10))) {
            state_=State::OPTIONAL_TX; guard_until_=now+GUARD;
            io_.event("UAV_RX",UAV,poll_seq_); return true;
        }
        if(state_==State::WAIT_ACK && p[3]==ACK && p[6]==2 && p[7]==WAYPOINT &&
           sequence(p)==sequence(inflight_.data)) {
            finish_command(p[8]==0?"ACK":"NACK");
            state_=State::OPTIONAL_TX; guard_until_=now+GUARD; return true;
        }
        return false;
    }
    // tx_result: 0 pending, 1 TxDone. A timeout forcibly stops RF before advancing.
    void tick(uint32_t now,int tx_result=0) {
        if(!enabled_) return;
        if(busy_) {
            if(tx_result==1 || due(now,tx_until_)) {
                bool ok=tx_result==1;
                if(!ok) { io_.abort(); io_.event("TX_TIMEOUT",0,0); }
                busy_=false; guard_until_=now+GUARD;
                after_tx(now,ok);
            } else return;
        }
        if(due(now,next_)) {
            // Never advance using now+PERIOD: keep the original phase on misses/wrap.
            uint32_t missed=(now-next_)/PERIOD;
            if(missed) io_.event("MISSED_CYCLES",0,uint16_t(missed));
            next_+=(missed+1)*PERIOD; ++cycle_;
            for(int i=0;i<4;++i) { active_[i]=ready_[i]; ready_[i].size=0; }
            frame_=0; offset_=0; state_=State::UGV_RTCM_TX;
            io_.event("CYCLE",0,uint16_t(cycle_));
        }
        if(!due(now,guard_until_)) return;
        switch(state_) {
        case State::UGV_RTCM_TX: send_rtcm(now); break;
        case State::UGV_GRANT_TX:
            if(!fits(now,budget(1)+RESPONSE+GUARD)) { state_=State::IDLE; break; }
            packet_[0]=0xA3; start(now,1); break;
        case State::WAIT_UGV:
            if(due(now,wait_until_)) { io_.event("UGV_TIMEOUT",UGV,0); state_=State::UAV_REQUEST_TX; }
            break;
        case State::UAV_REQUEST_TX:
            if(!fits(now,budget(HEADER)+UAV_RESPONSE+GUARD)) { state_=State::IDLE; io_.event("UAV_DEFERRED",UAV,0); break; }
            ++poll_seq_; header(packet_,GCS,UAV,POLL,poll_seq_,0); start(now,HEADER); break;
        case State::WAIT_UAV:
            if(due(now,wait_until_)) { io_.event("UAV_TIMEOUT",UAV,poll_seq_); state_=State::OPTIONAL_TX; }
            break;
        case State::OPTIONAL_TX: send_optional(now); break;
        case State::WAIT_ACK:
            if(due(now,wait_until_)) {
                if(inflight_.attempts>=2) finish_command("ACK_TIMEOUT");
                else io_.event("RETRY_PENDING",UAV,sequence(inflight_.data));
                state_=State::OPTIONAL_TX;
            }
            break;
        default: break;
        }
    }
private:
    IO io_; State state_=State::IDLE;
    Frame staging_[4],ready_[4],active_[4]; Command commands_[2],inflight_;
    bool busy_=false,enabled_=false; uint16_t preamble_=8,poll_seq_=0;
    uint32_t next_=0,tx_until_=0,wait_until_=0,guard_until_=0,cycle_=0;
    uint8_t frame_=0,rtcm_seq_=0,packet_[128]; uint16_t offset_=0,last_data_=0;
    int command_slot_=0;
    uint32_t budget(uint16_t n) const { return airtime(n,preamble_)+20+GUARD; }
    bool fits(uint32_t now,uint32_t ms) const { return int32_t(next_-now)>int32_t(ms); }
    void start(uint32_t now,uint8_t n) {
        tx_until_=now+airtime(n,preamble_)+20; busy_=true; io_.start(packet_,n);
    }
    uint32_t frame_budget(uint16_t n) const {
        if(n<=126) return budget(n+2);
        uint32_t sum=0; for(uint16_t i=0;i<n;i+=124) sum+=budget((n-i>124?124:n-i)+4);
        return sum;
    }
    void send_rtcm(uint32_t now) {
        while(frame_<4) {
            Frame &f=active_[frame_];
            if(!f.size) { ++frame_; continue; }
            if(offset_==0) {
                if(now-f.at>FRESH || !fits(now,frame_budget(f.size)+budget(1)+RESPONSE+GUARD)) {
                    io_.event(now-f.at>FRESH?"RTCM_STALE":"RTCM_OVER_BUDGET",UGV,frame_);
                    ++frame_; continue; // discard only a whole frame, never arbitrary bytes
                }
                ++rtcm_seq_;
            }
            if(f.size<=126) {
                packet_[0]=0xA1; packet_[1]=rtcm_seq_; last_data_=f.size;
                memcpy(packet_+2,f.data,f.size); start(now,f.size+2);
            } else {
                last_data_=f.size-offset_>124?124:f.size-offset_;
                packet_[0]=0xA2; packet_[1]=rtcm_seq_; packet_[2]=offset_/124; packet_[3]=(f.size+123)/124;
                memcpy(packet_+4,f.data+offset_,last_data_); start(now,last_data_+4);
            }
            return;
        }
        state_=State::UGV_GRANT_TX;
    }
    void finish_command(const char *status) {
        uint16_t seq=sequence(inflight_.data); uint8_t node=inflight_.data[2];
        if(commands_[command_slot_].size && sequence(commands_[command_slot_].data)==seq)
            commands_[command_slot_].size=0;
        io_.event(status,node,seq);
    }
    void send_optional(uint32_t now) {
        for(int i=0;i<2;++i) {
            Command &c=commands_[i];
            if(!c.size || c.cycle==cycle_) continue; // at most one attempt/node/cycle
            uint8_t n=i==0?uint8_t(c.data[6]+2):c.size;
            uint32_t reserve=budget(n)+(i==0?GUARD:UAV_RESPONSE+GUARD);
            if(!fits(now,reserve)) continue;
            c.cycle=cycle_; ++c.attempts; inflight_=c; command_slot_=i;
            if(i==0) { packet_[0]=GCS; packet_[1]=0xF2; memcpy(packet_+2,c.data+HEADER,c.data[6]); }
            else memcpy(packet_,c.data,c.size);
            state_=State::OPTIONAL_TX; start(now,n); return;
        }
        state_=State::IDLE;
    }
    void after_tx(uint32_t now,bool ok) {
        switch(state_) {
        case State::UGV_RTCM_TX:
            offset_+=last_data_;
            if(!ok || offset_>=active_[frame_].size) { ++frame_; offset_=0; }
            break;
        case State::UGV_GRANT_TX:
            io_.receive(); io_.event(ok?"UGV_GRANT_DONE":"UGV_GRANT_FAILED",UGV,0);
            // Even an unobserved TxDone may have been received by the UGV.
            state_=State::WAIT_UGV; wait_until_=now+RESPONSE; break;
        case State::UAV_REQUEST_TX:
            io_.receive(); state_=State::WAIT_UAV; wait_until_=now+UAV_RESPONSE; break;
        case State::OPTIONAL_TX:
            io_.receive();
            if(command_slot_==0) { finish_command(ok?"TX_ONLY":"TX_FAILED"); state_=State::OPTIONAL_TX; }
            else { state_=State::WAIT_ACK; wait_until_=now+UAV_RESPONSE; }
            break;
        default: break;
        }
    }
};
} // namespace master
