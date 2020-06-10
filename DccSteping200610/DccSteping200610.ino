//--------------------------------------------------------------------------------
// Smile Stepping Motor Decoder
// Board:AYA020-2
// [DccSteping200528.ino]
// Copyright (c) 2020 Ayanosuke(Maison de DCC)
// By aya.
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

// DCC Function Decoder Rev 2 for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
// http://dcc.client.jp
// http://ayabu.blog.shinobi.jp

// http://radiopench.blog96.fc2.com/blog-entry-579.html
// https://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.2.2_decoder_cvs_2012.07.pdf
// http://www7b.biglobe.ne.jp/~robe/cpphtml/html03/cpp03010.html

#include "NmraDcc.h"
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below
//#include "SoftwareSerial.h"

//SoftwareSerial mySerial(0,0); // RX, TX

//各種設定、宣言

#define DECODER_ADDRESS 3
//#define DCC_ACK_PIN 0   // Atiny85 PB0(5pin) if defined enables the ACK pin functionality. Comment out to disable.

#define AP 0            // Atiny85 PB4(3pin) A+
#define AN 3            // Atiny85 PB1(6pin) A-
#define BP 1            // Atint85 PB0(5pin) B+
#define BN 4            // Atiny85 PB3(2pin) B-

#define F0 0
#define F1 1
#define F2 2
#define F3 3
#define F4 4
#define F5 5
#define F6 6
#define F7 7
#define F8 8
#define F9 9
#define F10 10
#define F11 11
#define F12 12

#define ON 1
#define OFF 0

#define CV_F0 33
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_47_STEPING_SPEED 47
#define CV_48_STEPING_COUNTH 48
#define CV_49_STEPING_COUNTL 49
#define CV_50_PHASE 50
#define CV_51_DIR 51
#define CV_52_ZERO 52
#define CV_53_ZERO_SPEED 53
#define CV_54_ZERO_COUNTH 54
#define CV_55_ZERO_COUNTL 55

#define MAN_ID_NUMBER 166  /* Manufacture ID */

// O1-O4 の ON/OFFステータス
uint8_t State_O[6][2] = {0,0 , 0,0 , 0,0 , 0,0 , 0,0 ,0,0 }; // [0][n]は未使用

unsigned int pos = 32000;
void PMdrive1phase(unsigned int); 
void PMdrive2phase(unsigned int); 
void PMdrive12phase(unsigned int); 

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

//Task Schedule
unsigned long gPreviousL5 = 0;

//進行方向
uint8_t gState_Fn[13][2];

//CV related
uint8_t gCV1_SAddr = 3; 
uint8_t gCVx_LAddr = 3;
uint8_t gCV47_SteepingSpeed = 10; // 10 = 10msec制御  1msecも動いた
uint16_t gCV48_SteepingCount = 255; // 21で1回転くらい
uint8_t gCV50_phase = 1;          // pwm パターン
uint8_t gCV51_dir = 0;            // 方向
uint8_t _dir = 0;            // 方向
uint8_t gCV52_Zero = 0;            // 0点補正有効
uint8_t gCV53_ZeroSpeed = 10;       // 0スピード
uint16_t gCV54_ZeroCount = 255; // 21で1回転くらい


//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

struct CVPair {
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS}, // CV01
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},               // CV09 The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},          // CV17 XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},          // CV18 YY in the XXYY address
  {CV_29_CONFIG, 2 },                                // CV29 Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_F0 ,0}, //34
  {CV_F1 ,1}, //35
  {CV_F2 ,2}, //36
  {CV_F3 ,3}, //37
  {CV_F4 ,4}, //38
  {CV_F5 ,5},
  {CV_F6 ,0},
  {CV_F7 ,0},
  {CV_F8 ,0},
  {CV_F9 ,0},
  {CV_F10 ,0},
  {CV_F11 ,0},
  {CV_F12 ,0},
  {CV_47_STEPING_SPEED,10},
  {CV_48_STEPING_COUNTH,0},
  {CV_49_STEPING_COUNTL,255},
  {CV_50_PHASE, 1},
  {CV_51_DIR, 0},
  {CV_52_ZERO, 0},
  {CV_53_ZERO_SPEED,10},
  {CV_54_ZERO_COUNTH,0},
  {CV_55_ZERO_COUNTL,255},
};

void(* resetFunc) (void) = 0;  //declare reset function at address 0
//void LightMes( char,char );
//void pulse(void);
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

void notifyDccReset(uint8_t hardReset );

//uint16_t limitSpeed(uint16_t inSpeed);

void notifyCVResetFactoryDefault()
{
//mySerial.println("CVres");    
  //When anything is writen to CV8 reset to defaults.

  resetCVToDefault();
  //Serial.println("Resetting...");
  delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

  resetFunc();
};

//------------------------------------------------------------------
// Resrt
//------------------------------------------------------------------
void notifyDccReset(uint8_t hardReset )
{
//mySerial.println("DccRes");    
  digitalWrite(AP,LOW);
  digitalWrite(AN,LOW);
  digitalWrite(BP,LOW);
  digitalWrite(BN,LOW);

//  gState_F0 = 0; // ADD
//  gState_F1 = 0;
//  gState_F2 = 0;
//  gState_F3 = 0; 
//  gState_F4 = 0;
//  gState_F5 = 0;

//  resetFunc(); // <--- DEL    CV書き換え出来なくなってしまうのでコメント
  
}

//------------------------------------------------------------------
// CVをデフォルトにリセット
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
}

extern void	   notifyCVChange( uint16_t CV, uint8_t Value) {
  //CVが変更されたときのメッセージ
  //Serial.print("CV ");
  //Serial.print(CV);
  //Serial.print(" Changed to ");
  //Serial.println(Value, DEC);
};


//------------------------------------------------------------------
// CV Ack
//------------------------------------------------------------------
void notifyCVAck(void)
{
  //Serial.println("notifyCVAck");
  digitalWrite(AP,HIGH);
  digitalWrite(AN,LOW);
  digitalWrite(BP,HIGH);
  digitalWrite(BN,LOW);

  delay( 6 );

  digitalWrite(AP,LOW);
  digitalWrite(AN,LOW);
  digitalWrite(BP,LOW);
  digitalWrite(BN,LOW);
}

//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
  TCCR1 = 0<<CTC1 | 0<<PWM1A | 0<<COM1A0 | 0<<CS10;

//  mySerial.begin(9600); // ソフトウェアシリアルの初期化
//  mySerial.println("SDecoder");    
  pinMode(AP, OUTPUT);
  pinMode(AN, OUTPUT);
  pinMode(BP, OUTPUT);
  pinMode(BN, OUTPUT);

  //DCCの応答用負荷ピン
#if defined(DCCACKPIN)
  //Setup ACK Pin
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, 0);
#endif

#if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
  if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ) {	 //if eeprom has 0xFF then assume it needs to be programmed
    //Serial.println("CV Defaulting due to blank eeprom");
    notifyCVResetFactoryDefault();

  } else {
    //Serial.println("CV Not Defaulting");
  }
#else
  //Serial.println("CV Defaulting Always On Powerup");
  notifyCVResetFactoryDefault();
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0); // Atiny85 7pin(PB2)をDCC_PULSE端子に設定

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_NUMBER, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );

  //Reset task
  gPreviousL5 = millis();

  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS );
  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );
  gCV47_SteepingSpeed = Dcc.getCV( CV_47_STEPING_SPEED ) ;
  if(gCV47_SteepingSpeed == 0)
    gCV47_SteepingSpeed = 1;
  gCV48_SteepingCount = Dcc.getCV( CV_48_STEPING_COUNTH )<<8 | Dcc.getCV( CV_49_STEPING_COUNTL) ;
  gCV50_phase = Dcc.getCV( CV_50_PHASE ) ;
  gCV51_dir = Dcc.getCV( CV_51_DIR ) ;
  gCV52_Zero = Dcc.getCV( CV_52_ZERO ) ;
  gCV53_ZeroSpeed = Dcc.getCV( CV_53_ZERO_SPEED ) ;       // 0スピード
  if(gCV53_ZeroSpeed == 0)
    gCV53_ZeroSpeed = 1;
  gCV54_ZeroCount = Dcc.getCV( CV_54_ZERO_COUNTH )<<8 | Dcc.getCV( CV_55_ZERO_COUNTL ) ;

  gState_Fn[0][1] = Dcc.getCV( CV_F0 );
  gState_Fn[1][1] = Dcc.getCV( CV_F1 );
  gState_Fn[2][1] = Dcc.getCV( CV_F2 );
  gState_Fn[3][1] = Dcc.getCV( CV_F3 );
  gState_Fn[4][1] = Dcc.getCV( CV_F4 );
  gState_Fn[5][1] = Dcc.getCV( CV_F5 );
  gState_Fn[6][1] = Dcc.getCV( CV_F6 );
  gState_Fn[7][1] = Dcc.getCV( CV_F7 );
  gState_Fn[8][1] = Dcc.getCV( CV_F8 );
  gState_Fn[9][1] = Dcc.getCV( CV_F9 );
  gState_Fn[10][1] = Dcc.getCV( CV_F10 );
  gState_Fn[11][1] = Dcc.getCV( CV_F11 );
  gState_Fn[12][1] = Dcc.getCV( CV_F12 );

  for(int i=0;i<=12;i++){
    if(gState_Fn[i][1] != 0)
        State_O[ gState_Fn[i][1] ][0] = i;  // State_O に ファンクッション番号を格納
  }
}

//---------------------------------------------------------------------
// Arduino main loop
//---------------------------------------------------------------------
void loop() {
//  mySerial.print("*");    
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( (millis() - gPreviousL5) >= gCV47_SteepingSpeed) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    Steeper_Control();
    gPreviousL5 = millis();
  }
}


void cwP() {          // CW方向へ1パルス
  pos++;
//    mySerial.println("+");
  switch(gCV50_phase){
    case 1:
            PMdrive1phase(pos);    
            break;
    case 2:
            PMdrive2phase(pos);    
            break;
    case 3:
            PMdrive12phase(pos);    
            break;
    default:
            PMdrive1phase(pos);    
            break;        
  }
}
 
void ccwP() {         // CCW方向へ1パルス
  pos--;
//    mySerial.println("-");
  switch(gCV50_phase){
    case 1:
            PMdrive1phase(pos);    
            break;
    case 2:
            PMdrive2phase(pos);    
            break;
    case 3:
            PMdrive12phase(pos);    
            break;
    default:
            break;        
  }
}


//---------------------------------------------------------------------
// Steeping Motor control Task (10Hz:10ms)
//---------------------------------------------------------------------
void Steeper_Control()
{
  enum{
      ST_STANDABY = 0,
      ST_IDLE,
      ST_UP,
      ST_UP_HOLD,
      ST_DOWN,
      ST_MUP,
      ST_MDOWN,
      ST_ZERO,
      ST_ZERO_RUN,
  };
  
  static int state = ST_STANDABY;
  static int _i = 0;
  static char spr = 0;
  static char zf = 0;
  
  switch(state){
    case ST_STANDABY: // 電源投入直後
                      if(gCV52_Zero == 1){      // CV52のゼロ点有効になっている時0点処理を行う
                        state = ST_ZERO;  
                      } else {
                        state = ST_IDLE;                        
                      }
                      break;                    
    case ST_IDLE:     // アイドルステート
                      if(State_O[1][1] == ON){  // ステッピングモータ ON/OFF
                         state = ST_UP;
                         _i = 0;
                      }
                      if(State_O[2][1] == ON){  // ステッピングモータ UP
                         state = ST_MUP; 
                      }
                      if(State_O[3][1] == ON){  // ステッピングモータ DOWN
                         state = ST_MDOWN;
                      }

                      if(gCV51_dir == ON){
                        _dir = ON;
                        if(State_O[4][1] == ON){  // OFF:正転、ON:反転
                        _dir = OFF;
                        }
                      } else {
                        _dir = OFF;
                        if(State_O[4][1] == ON){  // OFF:正転、ON:反転
                        _dir = ON;
                        }
                      }

                      if(State_O[5][1] == ON && zf == 0){  // 0点制御
                         state = ST_ZERO;
                      } else if(State_O[5][1] == OFF){ 
                        zf = 0;
                      }
                        
                      break;
    case ST_UP:       // ステッピングモータUP制御
                      _i++;
                      if(_i >= (int)gCV48_SteepingCount){ // CV48まで達したら停止
                        digitalWrite(AP, LOW);
                        digitalWrite(AN, LOW);
                        digitalWrite(BP, LOW);
                        digitalWrite(BN, LOW);
                        state = ST_UP_HOLD;
                        break;
                      }
                      if(_dir == OFF ){  // CV51の極性指定によりup/down
                        cwP();
                      } else {
                        ccwP();
                      }
                      break;
    case ST_UP_HOLD:  // OFFコマンドまち
                    if(State_O[1][1] == OFF){
                      _i = 0;
                      state = ST_DOWN;
                    }
                    break;
    case ST_DOWN:     // ステッピングモータDOWN制御
                    _i++;
                    if(_i >= (int)gCV48_SteepingCount){ // CV48まで達したら停止
                        digitalWrite(AP, LOW);
                        digitalWrite(AN, LOW);
                        digitalWrite(BP, LOW);
                        digitalWrite(BN, LOW);
                      state = ST_IDLE;
                      break;
                    }
                      if(_dir == OFF ){  // CV51の極性指定によりup/down
                        ccwP();
                      } else {
                        cwP();
                      }
                    break;
    case ST_MUP:      // コマンドONの時だけUP制御
                      if(_dir == OFF ){
                        cwP();
                      } else {
                        ccwP();                  
                      }
                      
                      if(State_O[2][1] == OFF){ // コマンドOFFで停止
                        digitalWrite(AP, LOW);
                        digitalWrite(AN, LOW);
                        digitalWrite(BP, LOW);
                        digitalWrite(BN, LOW);
                      state = ST_IDLE;
                      }
                      break;
    case ST_MDOWN:      // コマンドONの時だけDOWN制御
                      if(_dir == OFF ){
                        ccwP();
                      } else {
                        cwP();
                      }
                                         
                      if(State_O[3][1] == OFF){ // コマンドOFFで停止
                        digitalWrite(AP, LOW);
                        digitalWrite(AN, LOW);
                        digitalWrite(BP, LOW);
                        digitalWrite(BN, LOW);
                      state = ST_IDLE;
                      }
                      break;
    case ST_ZERO:       // 0点制御事前準備
                      spr = gCV47_SteepingSpeed;              // 強引な処理
                      gCV47_SteepingSpeed = gCV53_ZeroSpeed;
                      _i = 0;
                      state = ST_ZERO_RUN;
                      break;
    case ST_ZERO_RUN:   // 0点制御
                    _i++;
                    if(_i >= (int)gCV54_ZeroCount){
                        digitalWrite(AP, LOW);
                        digitalWrite(AN, LOW);
                        digitalWrite(BP, LOW);
                        digitalWrite(BN, LOW);
                      gCV47_SteepingSpeed = spr;
                      zf = 1;
                      state = ST_IDLE;
                      break;
                    }
                      if(_dir == OFF ){
                        ccwP();
                      } else {
                        cwP();
                      }
                    break;                      
    default:
                    break;
     
    }
}



void PMdrive1phase(unsigned int n) {     // 指定状態にコイルを励磁
  int phase;
  phase = n % 4;                   // 位相を求め
  switch (phase) {                 // 相当する状態に通電
    case 0:
      digitalWrite(AP, HIGH); // A+
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, LOW);
      break;
    case 1:
      digitalWrite(AP, LOW);
      digitalWrite(AN, LOW);
      digitalWrite(BP, HIGH);  // B+
      digitalWrite(BN, LOW);
      break; 
     case 2:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH); // A-
      digitalWrite(BP, LOW);
      digitalWrite(BN, LOW);
      break;
    case 3:
      digitalWrite(AP, LOW);
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);   // B-
      break;
    default:
      break;
  }
}

void PMdrive2phase(unsigned int n) {     // 指定状態にコイルを励磁
  int phase;
  phase = n % 4;                   // 位相を求め
  switch (phase) {                 // 相当する状態に通電
    case 0:
      digitalWrite(AP, HIGH); // A+
      digitalWrite(AN, LOW);
      digitalWrite(BP, HIGH);
      digitalWrite(BN, LOW);
      break;
    case 1:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH);
      digitalWrite(BP, HIGH);  // B+
      digitalWrite(BN, LOW);
      break; 
     case 2:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH); // A-
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);
      break;
    case 3:
      digitalWrite(AP, HIGH);
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);   // B-
      break;
    default:
      break;
  }
}


void PMdrive12phase(unsigned int n) {     // 指定状態にコイルを励磁
  int phase;
  phase = n % 8;                   // 位相を求め
  switch (phase) {                 // 相当する状態に通電
    case 0:
      digitalWrite(AP, HIGH); // A+
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, LOW);
      break;
    case 1:
      digitalWrite(AP, HIGH);
      digitalWrite(AN, LOW);
      digitalWrite(BP, HIGH);  // B+
      digitalWrite(BN, LOW);
      break; 
     case 2:
      digitalWrite(AP, LOW);
      digitalWrite(AN, LOW); // A-
      digitalWrite(BP, HIGH);
      digitalWrite(BN, LOW);
      break;
    case 3:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH);
      digitalWrite(BP, HIGH);
      digitalWrite(BN, LOW);   // B-
      break;
    case 4:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH);
      digitalWrite(BP, LOW);
      digitalWrite(BN, LOW);   // B-
      break;
    case 5:
      digitalWrite(AP, LOW);
      digitalWrite(AN, HIGH);
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);   // B-
      break;
    case 6:
      digitalWrite(AP, LOW);
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);   // B-
      break;
    case 7:
      digitalWrite(AP, HIGH);
      digitalWrite(AN, LOW);
      digitalWrite(BP, LOW);
      digitalWrite(BN, HIGH);   // B-
      break;
    default:
      break;
  }
}



//DCC速度信号の受信によるイベント
//extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  
//  if ( gDirection != ForwardDir) // old NMRA
//  if ( gDirection != Dir )
//  {
//  gDirection = ForwardDir; // old NMRA
//    gDirection = Dir;
//  }
//  gSpeedRef = Speed;
}




//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
//extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
extern void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{

#if 0
   mySerial.print(FuncGrp,DEC);
   mySerial.print(",");
   mySerial.println(FuncState,DEC);
#endif      
  switch(FuncGrp){
    case FN_0_4:
      if( gState_Fn[F0][0] != (FuncState & FN_BIT_00)){     // 要素1:ファンクッション番号  要素2[0]:ファンクッションの状態
        gState_Fn[F0][0] = (FuncState & FN_BIT_00);
        if(gState_Fn[F0][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F0][1] ][1] = gState_Fn[F0][0] ? ON : OFF; // gState_Fn[F0][0]の値を見て、ON/OFFを決める
      }
      
      if( gState_Fn[F1][0] != (FuncState & FN_BIT_01)){
        gState_Fn[F1][0] = (FuncState & FN_BIT_01);
        if(gState_Fn[F1][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F1][1] ][1] = gState_Fn[F1][0] ? ON : OFF;
      }
      
      if( gState_Fn[F2][0] != (FuncState & FN_BIT_02)){
        gState_Fn[F2][0] = (FuncState & FN_BIT_02);
        if(gState_Fn[F2][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F2][1] ][1] = gState_Fn[F2][0] ? ON : OFF;
      }
      
      if( gState_Fn[F3][0]!= (FuncState & FN_BIT_03)){
        gState_Fn[F3][0] = (FuncState & FN_BIT_03);
        if(gState_Fn[F3][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F3][1] ][1] = gState_Fn[F3][0] ? ON : OFF;
      }
      
      if( gState_Fn[F4][0] != (FuncState & FN_BIT_04)){
        gState_Fn[F4][0] = (FuncState & FN_BIT_04);
        if(gState_Fn[F4][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F4][1] ][1] = gState_Fn[F4][0] ? ON : OFF;
      }
    break;

  case FN_5_8:
    if( gState_Fn[F5][0] != (FuncState & FN_BIT_05)){
        gState_Fn[F5][0] = (FuncState & FN_BIT_05);
        if(gState_Fn[F5][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F5][1] ][1] = gState_Fn[F5][0] ? ON : OFF;
    }
    
    if( gState_Fn[F6][0] != (FuncState & FN_BIT_06)){
        gState_Fn[F6][0] = (FuncState & FN_BIT_06);
        if(gState_Fn[F6][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F6][1] ][1] = gState_Fn[F6][0] ? ON : OFF;
    } 
    
    if( gState_Fn[F7][0] != (FuncState & FN_BIT_07)){
        gState_Fn[F7][0] = (FuncState & FN_BIT_07);
        if(gState_Fn[F7][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F7][1] ][1] = gState_Fn[F7][0] ? ON : OFF;
    }
    
    if( gState_Fn[F8][0] != (FuncState & FN_BIT_08)){
        gState_Fn[F8][0] = (FuncState & FN_BIT_08);
        if(gState_Fn[F8][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F8][1] ][1] = gState_Fn[F8][0] ? ON : OFF;
    }
    break;

  case FN_9_12:
    if( gState_Fn[F9][0] != (FuncState & FN_BIT_09)) {
        gState_Fn[F9][0] = (FuncState & FN_BIT_09);
        if(gState_Fn[F9][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F9][1] ][1] = gState_Fn[F9][0] ? ON : OFF;
    }
    if( gState_Fn[F10][0] != (FuncState & FN_BIT_10)){
        gState_Fn[F10][0] = (FuncState & FN_BIT_10);
        if(gState_Fn[F10][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F10][1] ][1] = gState_Fn[F10][0] ? ON : OFF;
    } 
    if( gState_Fn[F11][0] != (FuncState & FN_BIT_11)) {
        gState_Fn[F11][0] = (FuncState & FN_BIT_11);
        if(gState_Fn[F11][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F11][1] ][1] = gState_Fn[F11][0] ? ON : OFF;
    }
    if( gState_Fn[F12][0] != (FuncState & FN_BIT_12)) {
        gState_Fn[F12][0] = (FuncState & FN_BIT_12);
        if(gState_Fn[F12][1] !=0)                            // O1-O4が割り当てられている
          State_O[ gState_Fn[F12][1] ][1] = gState_Fn[F12][0] ? ON : OFF;
    }
    break;  

  default:
    break;
  }
}
