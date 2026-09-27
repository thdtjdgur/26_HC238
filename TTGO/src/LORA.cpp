#include "lora.h"
#include "oled.h"
extern bool master_on_rx(const uint8_t *packet, uint8_t length);



void lora_write(uint8_t adr, uint8_t data); // lora_setup에서 써서 미리 선언

SPISettings droneset(8000000, MSBFIRST, SPI_MODE0); //클럭속도(8MHz,  MSB먼저, 모드0 : 라이징에서 읽음) --> 클럭속도는 lora칩은 10Mhz까지 가능(Fsck)
uint8_t txflag = 0;
static int cnt = 0;
static int snd = 0;

uint8_t rxflag = 0;
int rcv = 0;
int r_byte = 0;

void lora_setup (void)
{
    // Serial은 PC 패킷 파서가 RX 버퍼 크기를 먼저 정해야 하므로 main의 setup에서 연다.
    SPI.begin(SCLK, MISO, MOSI, CS); // 이 함수 하나로 인자로 넘겨준 핀을 spi의 sck,miso,mosi,ss에 설정
    pinMode(CS, OUTPUT); // cs 핀
    //pinMode(RST, OUTPUT);
    digitalWrite(CS,HIGH); // 기본 high로 설정

    lora_write(0x09, 0x8F); //출력 관련 세팅 : PA_BOOST 핀(1)--> 안테나쪽으로 출력연결+고출력, MAX POWER : 000 , OUTPUTPOWER : 1111(17dBm)  --> 1000 1111 = 0x8F
    lora_write(0x0B, 0x31); // 과전류 방지 전류 보호회로 세팅(OCP) --> 0011 0001 --> 140mA로 제한

    lora_write(0x11,0x00); //인터럽트 마스크 안해줘서 다 허용(tx 인터럽트 mask 안해주는겸)
    lora_write(0x12, 0xFF); // 안전빵으로 인터럽트 flag 한번씩 다 clear
}

void lora_write(uint8_t adr, uint8_t data) //레지스터 데이터 길이가 1byte일때만(single access 방식)
{
    SPI.beginTransaction(droneset);                                         // 이거 굳이 매번해야하나?
    digitalWrite(CS,LOW); //SPI 시작
    SPI.transfer((write_bit)|(adr)); //읽기 + 주소 전달
    SPI.transfer(data); //transfer는 지금 전이중이니깐 MOSI로 1bit보내면 MISO로 1bit온다. 그래서 이 돌아온 1BIT가 저 함수 구조에 의해 tranfer함수의 반환값으로 돌아온다(1byte식)
    digitalWrite(CS,HIGH);
    SPI.endTransaction();
    //Serial.println("write complete\n");
}

void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3) //레지스터 길이가 2byte보다 길어서 한번에 BURST access방식으로 레지스터에 넣어줌(주소가 이어진 레지스터의 상황에서만 가능)
{
    SPI.beginTransaction(droneset);
    digitalWrite(CS,LOW);
    SPI.transfer((write_bit)|(adr));
    SPI.transfer(data1);
    SPI.transfer(data2);
    if (data3!=no_more) SPI.transfer(data3); // 3개까지 burst하고싶지않을때 no_more 인자 주면됨
    digitalWrite(CS,HIGH);
    SPI.endTransaction();
    //Serial.println("write BURST complete\n");
}

uint8_t lora_read(uint8_t adr)
{
    //delay(10); // write하고 좀 시간 delay줌(쓸시간 줘야함)
    SPI.beginTransaction(droneset);
    digitalWrite(CS,LOW); //SPI 시작
    SPI.transfer((read_bit)|(adr)); //읽기 + 주소 전달
    uint8_t val = SPI.transfer(0X00); //transfer는 지금 전이중이니깐 MOSI로 1bit보내면 MISO로 1bit온다. 그래서 이 돌아온 1BIT가 저 함수 구조에 의해 tranfer함수의 반환값으로 돌아온다(1byte식)
    digitalWrite(CS,HIGH);
    SPI.endTransaction();
    //Serial.printf("read data: 0x%02X\n", val);

    return val;
}

void lora_freq(void) //주파수 922.1MHz로 설정  ( 922100000 / 61.03515625 = 15107686 = E68666)
{
    //주의ㅣ*** : 주파수(LoRa Frequency)를 변경하려면, 칩은 반드시 Sleep 모드나 Standby 모드 상태여야만 가능."
    lora_write(0x01,sleep);
    //sleep이랑 주파수설정 통합했음
    lora_write_burst(0x06,0XE6,0X86,0X66); // 주소, RegFrfMsb, RegFrfMid, RegFrfLsb (burst access로 함)
    //lora_read(0x06);
    //lora_read(0x07);
    //lora_read(0x08);
}

void packet_set(void) //LORA링크 성격 설정 +  패킷관련 설정 - 이 레지스터들 쓸때 무조건 SLEEP이나 STDBY상태여야함
{
    //lora_write(0x01,sleep);
    lora_write(0x1D,0X72);  //-> 0111 0010 = 0X72 (BW(대역폭):125kHz, CR(부호화율):4/5, Explicit Header mode)

    lora_write(0x1E,0X74);  //LoRa의 속도,신뢰성,수신 동작 방식을 정하는 레지스터  - 이 레지스터 쓸때 무조건 SLEEP이나 STDBY상태여야함
    //RegModemConfig2(0x1E)_ -> 0111 0100 = 0X94 (SF->7(통신거리,속도),  통신방식 : 패킷으로(분석용x), PayLoad CRC : ON, RX TIMEOUT : 0(이건 일단은 0으로 나중에 최적화부분 비트여서))
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  tx관련



void fifo_set(void) //payload 전송전 세팅
{
    lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, fifo는 stanby에서만 접근 가능
    //Serial.printf("MODE : STANBY, \n\n");

    lora_write_burst(0x0E,0X80,0X00,no_more); //송수신데이터 저장 메모리 위치 설정
    //lora_write(0x0E,0X80);  //RegFifoTxBaseAddr : 송신 데이터(Tx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x80)
    //lora_write(0x0F,0X00);  //RegFifoRxBaseAddr : 수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
}

/*
void payload_write(void) // 실제 보낼 데이터 넣는 설정
{
    uint8_t packet_data[4] = {'w','a','s','d'};
    lora_write(0x22,sizeof(packet_data));  //RegPayloadLength : 보낼 데이터 크기 설정 + header에 싣는 payload 길이

    // 1. 포인터 초기화 (가장 중요!)
    lora_write(0x0D, 0x80); //RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정

    // 2. FOR문으로 데이터 밀어넣기
    for(int i = 0; i < 4; i++)
    {
        lora_write(0x00, packet_data[i]); // RegFifo : 포인터로 가르킨 주소 0x80에 보낼 데이터를 씀 -> 포인터가 알아서 1씩 증가함
    }

    lora_write(0x0D, 0x80); //위에서 값 쓰면서 포인터 증가해서 읽을라면 다시 처음 쓴 위치로 돌려야 순서대로 읽을수이씨음

    for(int i = 0; i < 4; i++)
    {
       snd = lora_read(0x00); // 0x00에 접근하면 자동 포인터 증가해서 읽기위해서 이렇게 짬
       Serial.printf("send info : %c\n",snd);
    }
}*/

void payload_write(const uint8_t *buf, uint8_t len) // 실제 보낼 데이터 넣는 설정
{
    lora_write(0x22,len);  //RegPayloadLength : 보낼 데이터 크기 설정 + header에 싣는 payload 길이
    //Serial.printf("send byte : %d\n",len);

    // 1. 포인터 초기화 (가장 중요!)
    lora_write(0x0D, 0x80); //RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정

    // 2. FOR문으로 데이터 밀어넣기

    for (uint8_t i = 0; i < len; i++)   // RegFifo : 포인터로 가르킨 주소 0x80에 보낼 데이터를 씀 -> 포인터가 알아서 1씩 증가함
    {
        lora_write(0x00, buf[i]);
    }

    // 예전 테스트에서는 FIFO를 다시 읽어 송신 데이터를 확인했지만, 실사용에서는
    // 그 시간만큼 다음 LoRa 패킷 처리가 늦어지므로 read-back 디버깅을 하지 않는다.
}


bool real_tx(uint32_t timeout_ms)
{
    lora_write(0x01,stanby); //fifo메모리는 sleep에서 초기화되서 무조건 sleep에서는 하면 안됨, 이제 접근해서 보내야하니 무조건 stanby로

    lora_write(0x12, 0xFF); //안전빵으로 들어올 때 인터럽트 플래그 한번 클리어 해줌 + txdone에 1써서 다시 인터럽트 플래그 클리어
    lora_write(0x0D, 0x80); // 포인터 위치 다시 돌림
    lora_write(0x01,tx); // 실제로 보냄
    uint32_t tx_start = millis();

    //Serial.printf("\n보내는중임\n\n");
    do //읽어서 이제 잘 보내졌는지 확인 ==> txdone은 0~7비트중 3비트 위치에 있음 그래서 잘 보내지면 탈출하고 플래그 다시 끄기ㅣ
    {
        txflag = lora_read(0x12); //txdone 인터럽트 켜지는지 계속 보기
        yield(); // 통신 오류로 WHILE문 안에 계속 갇혀있을떄 뻑가는거 방지용
        //ESP32 안에 사는 **'와치독(Watchdog)'**이라는 감시관이 하는 생각입니다. 얘는 CPU가 한 곳(while)에 오래 머물러 있으면 "시스템이 뻗었다(Freeze)"고 판단합니다.
        //루프 한 바퀴 돌 때마다 **"저 아직 안 죽었고, 열심히 일하는 중입니다!"**라고 감시관한테 보고하는 겁니다. 그러면 감시관이 "어, 그래? 살아있네." 하고 카운트다운(재부팅 타이머)을 다시 0초로 초기화해줍니다.
        cnt++;
        if((uint32_t)(millis() - tx_start) >= timeout_ms)
        {
            lora_write(0x01,stanby);
            lora_write(0x12, 0xFF);
            cnt = 0;
            return false;
        }
    }while(!(txflag & 0x08));
    //Serial.printf("일단 보냄 보냄 : %d\n",cnt);
    lora_write(0x12, 0x08); // TxDone bit만 clear
    cnt = 0 ;
    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rx_set(void)
{
	lora_write(0x01,stanby); //sleep,stanby모드여야한다*************
	lora_write(0x12, 0xFF); // rxdone 인터럽트 클리어

	lora_write(0x0F,0X00); //수신 데이터(Rx) 가 FIFO 메모리 어디부터 저장될지 정함(기본값: 0x00)
	lora_write(0x0D, 0x00); ////RegFifoAddrPtr : 데이터 읽고 쓸 위치 설정
	lora_write(0x40, 0x00); // DI0을 rxdone의 인터럽트로 쓰겠다 (rx 들오면 DI0핀에 인터럽트 발생)
	lora_write(0x01, 0x85); // 000 0101 -> 0x85(RXCONTINOUS 모드) -------> 이거 하면 바로 수신시작
}


void rx_read(void)
{
    uint8_t irq;
    uint8_t len;
    uint8_t current_addr;
    uint8_t rx_buf[255];   // LoRa로 받은 payload 임시 저장 버퍼

    // RegIrqFlags = 0x12
    irq = lora_read(0x12);

    // 아직 수신 완료나 CRC 오류가 없으면 RX 상태를 건드리지 않고 바로 돌아간다.
    if((irq & 0x60) == 0)
    {
        return;
    }

    // RxDone bit = bit6 = 0x40
    if(irq & 0x40)
    {
        // CRC error bit = bit5 = 0x20
        if(irq & 0x20)
        {
            // GUI가 사용하는 P 라인 형식으로 CRC 실패 사실만 전달한다.
            Serial.printf("P,%lu,%d,%d,0,\r\n",
                          (unsigned long)millis(),
                          lora_packet_rssi_dbm(), lora_packet_snr_x100());
            oled_show_rx_crc_error();
            lora_write(0x12, 0xFF); // IRQ clear
            rx_set();
            return;
        }

        // RegRxNbBytes = 0x13
        len = lora_read(0x13);

        // RegFifoRxCurrentAddr = 0x10
        current_addr = lora_read(0x10);

        // RegFifoAddrPtr = 0x0D
        // 수신된 패킷의 FIFO 시작 주소로 포인터 이동
        lora_write(0x0D, current_addr);

        // FIFO에서 수신 payload 읽어서 rx_buf에 저장
        for(uint8_t i = 0; i < len; i++)
        {
            // RegFifo = 0x00
            rx_buf[i] = lora_read(0x00);
        }

        // Restore RX before callback/logging. Only an expected slot response
        // is reported to the GUI; another node cannot advance the scheduler.
        rx_set();
        if(!master_on_rx(rx_buf, len)) return;
        // 핵심:
        // LoRa로 받은 payload를 기존 GUI가 읽는 ASCII P 라인으로 전송한다.
        // 예: P,123500,-55,825,1,FE00F301E7AC2016D7A0C54B
        Serial.printf("P,%lu,%d,%d,1,", (unsigned long)millis(),
                      lora_packet_rssi_dbm(), lora_packet_snr_x100());
        for(uint8_t i = 0; i < len; i++)
        {
            Serial.printf("%02X", rx_buf[i]);
        }
        Serial.print("\r\n");
        if(len >= 7 && rx_buf[0] == 0xA4 && rx_buf[1] == 0xFD && rx_buf[2] == 0xFF)
        {
            oled_update_uav_packet(rx_buf, len);
        }
        else if((len == 14 || len == 23) && rx_buf[0] == 0xFE)
        {
            oled_update_gps_packet(rx_buf, len);
        }
    }

    // IRQ clear
    lora_write(0x12, 0xFF);
    rx_set();
}

int16_t lora_current_rssi_dbm(void)
{
    // 922.1MHz는 HF port(779MHz 이상)이므로 RegRssiValue에서 157을 뺀다.
    return (int16_t)lora_read(0x1B) - 157;
}

int16_t lora_packet_rssi_dbm(void)
{
    return (int16_t)lora_read(0x1A) - 157;
}

int16_t lora_packet_snr_x100(void)
{
    // RegPktSnrValue는 signed 0.25dB 단위이므로 100배 dB 값은 25를 곱한다.
    const int8_t raw = (int8_t)lora_read(0x19);
    return (int16_t)raw * 25;
}
