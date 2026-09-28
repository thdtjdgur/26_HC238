#include <Arduino.h>
#include "lora.h"
#include "oled.h"
#include "rtcm3.h"
#include "master_scheduler.h"

// UART records are unchanged for RTCM: FF F4 len16 RTCM, FF F5 commit.
// FF F6 len8 A4... adds addressed commands; FF F7 queries firmware capability.
// FF F5 now commits the latest set; the radio scheduler sends A3 after actual TX.
static bool radio_receiving=false;
static void receive_mode() { rx_set(); radio_receiving=true; }
static void abort_tx() { lora_write(0x01,stanby); lora_write(0x12,0xFF); }
static void start_tx(const uint8_t *p,uint8_t n) {
    radio_receiving=false;
    abort_tx(); payload_write(p,n);
    lora_write(0x40,0x40); // DIO0 = TxDone; IRQ also polled without blocking
    lora_write(0x01,tx);
}
static void status(const char *event,uint8_t node,uint16_t seq) {
    Serial.printf("S,%lu,%s,%u,%u\r\n",(unsigned long)millis(),event,node,seq);
    if(!strcmp(event,"UGV_GRANT_DONE") || !strcmp(event,"UGV_GRANT_FAILED"))
        Serial.printf("D,%lu,%u\r\n",(unsigned long)millis(),!strcmp(event,"UGV_GRANT_DONE"));
}
static master::Scheduler scheduler({start_tx,receive_mode,abort_tx,status});
// LORA.cpp calls this only for a CRC-clean packet. Ignore out-of-slot packets.
bool master_on_rx(const uint8_t *p,uint8_t n) { return scheduler.received(p,n,millis()); }
static Rtcm3StreamParser parser;
enum PcState { ID,TYPE,LENGTH_LO,LENGTH_HI,RTCM,EXT_LENGTH,EXT_DATA,WP_COUNT,WP_DATA };
static PcState pc_state=ID;
static uint16_t remaining=0,length=0,legacy_seq=0;
static uint8_t buf[128],used=0,wanted=0;
static uint32_t last_byte=0,last_rssi=0;
static void reset_parser(bool discard) {
    pc_state=ID; remaining=0; used=0;
    if(discard) { parser.reset_pending(); scheduler.discard_staging(); }
}
static void on_frame(const uint8_t *p,uint16_t n,uint16_t type) {
    scheduler.stage(p,n,type,millis()); // parser/CRC24Q unchanged
}
static void pc_byte(uint8_t b) {
    last_byte=millis();
    switch(pc_state) {
    case ID: if(b==0xFF) pc_state=TYPE; break;
    case TYPE:
        if(b==0xF4) pc_state=LENGTH_LO;
        else if(b==0xF5) { scheduler.commit(millis()); reset_parser(false); }
        else if(b==0xF6) pc_state=EXT_LENGTH;
        else if(b==0xF7) { Serial.print("M,1,3000\r\n"); reset_parser(false); }
        else if(b==0xF2) pc_state=WP_COUNT;
        else if(b!=0xFF) reset_parser(false);
        break;
    case LENGTH_LO: length=b; pc_state=LENGTH_HI; break;
    case LENGTH_HI:
        length|=uint16_t(b)<<8; remaining=length;
        if(!length || length>4096) reset_parser(true); else pc_state=RTCM;
        break;
    case RTCM:
        parser.feed(&b,1,on_frame);
        if(--remaining==0) reset_parser(parser.pending_bytes()!=0);
        break;
    case EXT_LENGTH:
        wanted=b; used=0;
        if(b<7 || b>128) reset_parser(true); else pc_state=EXT_DATA;
        break;
    case EXT_DATA:
        buf[used++]=b;
        if(used==wanted) {
            if(!scheduler.command(buf,used)) status("BAD_COMMAND",0,0);
            reset_parser(false);
        }
        break;
    case WP_COUNT:
        if(!b || b>15) { reset_parser(false); break; }
        master::header(buf,master::GCS,master::UGV,master::WAYPOINT,++legacy_seq,1+8*b);
        buf[7]=b; used=8; wanted=8+8*b; pc_state=WP_DATA; break;
    case WP_DATA:
        buf[used++]=b;
        if(used==wanted) { scheduler.command(buf,used); reset_parser(false); }
        break;
    }
}
void setup() {
    Serial.setRxBufferSize(8192); Serial.begin(115200);
    delay(100); // startup only
    oled_init(); lora_setup(); lora_freq(); packet_set(); fifo_set();
    scheduler.set_preamble((uint16_t(lora_read(0x20))<<8)|lora_read(0x21));
    receive_mode(); Serial.print("M,1,3000\r\n");
}
void loop() {
    // Bounded UART work ensures a busy PC cannot starve TxDone and deadlines.
    for(unsigned i=0;i<256 && Serial.available()>0;++i) {
        int b=Serial.read(); if(b<0) break; pc_byte(uint8_t(b));
    }
    uint32_t now=millis();
    if(pc_state!=ID && uint32_t(now-last_byte)>=1500) {
        reset_parser(true); status("SERIAL_TIMEOUT",0,0);
    }
    if(radio_receiving) rx_read();
    int done=scheduler.transmitting() && (lora_read(0x12)&0x08) ? 1:0;
    scheduler.tick(millis(),done);
    now=millis();
    if(radio_receiving && uint32_t(now-last_rssi)>=50) {
        last_rssi=now;
        Serial.printf("R,%lu,%d\r\n",(unsigned long)now,lora_current_rssi_dbm());
    }
    yield();
}
