//このプログラムを使用する際には下記のライブラリを使用してください．
//Arduino基本ライブラリ
#include <Arduino.h>
//sbus用ライブラリ(bolderflight/sbus)
#include <sbus.h> //ver 8.1.4
//ESP32専用PWM用ライブラリ
#include <esp32-hal-ledc.h>
//BNO055用のライブラリ
#include <Adafruit_Sensor.h> //installed with Adafruit_BNO055
#include <Adafruit_BNO055.h> //ver 1.6.1
//FS3000用のライブラリ
#include <SparkFun_FS3000_Arduino_Library.h> //ver1.0.4

//センサーや制御の変数をまとめたもの
struct datastr{

  //IMU
  //加速度[m/s^2]
  float accX = 0.0F;
  float accY = 0.0F;
  float accZ = 0.0F;
  //角速度[deg/s]
  float gyroPitch = 0.0F;
  float gyroRoll = 0.0F;
  float gyroYaw = 0.0F;
  //オイラー角[deg]
  float pitch = 0.0F;
  float roll  = 0.0F;
  float yaw   = 0.0F;

  //対気速度[m/s]
  float velo = 0.0F;

  //目標姿勢角[deg]
  float pitch_r = 0.0F;
  float roll_r  = 0.0F;

  //PWMの入力カウント数[count]
  float pitch_u = 0.0;
  float roll_u = 0.0;
  int ESC_u = 0;
  int gear = 0;

  //時間[ms]
  long unsigned int time = 0;
  //ループごとの時間[ms]
  uint8_t deltaT = 0;

  //制御・マニュアル変更
  bool control_bool = 0;

  //ゲイン
  float Kp = 0.0; //比例制御
  float Kd = 0.07; //微分制御
  float Ki = 0.8; //積分制御
  //偏差
  float e = 0.0; //偏差そのまま
  float eint = 0.0; //偏差の累積
};

//上記のdatastr型の変数を宣言する
datastr data;

//IMUの値を取得するクラスbnoを宣言
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
//取得したセンサーの値を格納する変数を宣言
sensors_event_t bnodata;

//FS3000の値を取得するクラスspeedを宣言
FS3000 speed;

//sbus
/* SbusRx object on Serial1 */
bfs::SbusRx sbus_rx(&Serial1,33,14,true);
// channel, fail safe, and lost frames data
bfs::SbusData sbus_data;

//PWM properties
//サーボやESCはHIGHの時間が1msから2ms（または0.5ms-2.5ms）のPWMを受け付ける．
#define PWM_FREQ 125 //PWMの周期を8msとするためにPWM周波数を125Hzに設定する．
#define DUTY_RESOLUTION 14 //PWMの周期を11bit*8個に分割する(14bit=11bit*8)ことを決める．そうすると1ms当たり11bitになる．11bitはsbusのデータサイズと同じなのでsbus->PWMのカウント数のスケール変更が不要になる．
#define COUNT_LOW 2048 //PWMの最小カウント数．11bit=2048count=1msなので，ここを最小値とする．
#define COUNT_HIGH 4096 //PWMの最大カウント数．12bit=4096count=2msなので，ここを最大値とする．


//前ループの時間を格納する変数
unsigned long prev_time = 0;

/*-------------setup------------------*/
void setup() {
  
  //init PWM
  ledcSetup(1, PWM_FREQ, DUTY_RESOLUTION); //PWM1番chは125Hz=8msを14bitで分割
  ledcAttachPin(32, 1); //PWM1番chはGPIOI32番に設定
  ledcSetup(2, PWM_FREQ, DUTY_RESOLUTION); //PWM2番chは125Hz=8msを14bitで分割
  ledcAttachPin(25, 2); //PWM1番chはGPIOI25番に設定
  ledcSetup(3, PWM_FREQ, DUTY_RESOLUTION); //PWM3番chは125Hz=8msを14bitで分割
  ledcAttachPin(26, 3); //PWM1番chはGPIOI26番に設定
  ledcSetup(4, PWM_FREQ, DUTY_RESOLUTION); //PWM4番chは125Hz=8msを14bitで分割
  ledcAttachPin(27, 4); //PWM1番chはGPIOI27番に設定

  //PC等にデータを送信するシリアルチャンネルを設定する
  //baudrateを115200に設定する．boudrateは通信の周波数を決める．値が大きいほど通信が速いが，ノイズに弱くなる．
  //baudrateは通信する機器双方で同じ値にしないと，周波数が噛み合わなくて値が壊れる．
  Serial.begin(115200);
  //はじまるよって表示する
  Serial.println("starting...");
  //init i2c
  Wire.begin();
  Wire.setClock(400000);
  

  //sbusを起動
  sbus_rx.Begin();
  //IMUを起動
  bno.begin();
  //FS3000を起動
  speed.begin();
  speed.setRange(AIRFLOW_RANGE_15_MPS); //使用しているFS3000が何m/sまで取れるものか指定する．今回は15m/sまで使用できるFS3000-1015を使用．

  //いろんな設定を反映するために休憩1s
  delay(1000);

  //シリアルから送信するデータの順序を示す
  Serial.println("time[ms],airspeed[m/s],accX[m/s^s],accY[m/s^s],accZ[m/s^s],p[deg/s],q[deg/s],r[deg/s],phi[deg],theta[deg],psi[deg],ESC_u[count],pitch_u[count],roll_u[count],gain[-]");

}



/*----------control loop----------*/
void loop() {

  //IMUのデータを取得する．IMUのxyz軸が機体のxyz軸に合っているかは設置の仕方次第なので，ここで軸を取り替えるなどする．
  //IMUはできるだけ飛行機の軸に対して斜めにならないように配置する．斜めになると以下に書いてある座標系変換では対応できない．
  bno.getEvent(&bnodata, Adafruit_BNO055::VECTOR_GYROSCOPE); //角速度の取得
  data.gyroPitch =   bnodata.gyro.x;
  data.gyroRoll  =   bnodata.gyro.y;
  data.gyroYaw   =  -bnodata.gyro.z;
  
  bno.getEvent(&bnodata, Adafruit_BNO055::VECTOR_EULER); //オイラー角の姿勢を取得
  data.pitch =  -bnodata.orientation.z;
  data.roll  =  -bnodata.orientation.y;
  data.yaw   =   bnodata.orientation.x;

  bno.getEvent(&bnodata, Adafruit_BNO055::VECTOR_ACCELEROMETER); //加速度を取得（現在使われていない）
  data.accX =  -bnodata.acceleration.y;
  data.accY =  -bnodata.acceleration.x;
  data.accZ =   bnodata.acceleration.z;
  
  //FS3000から対気速度を取得
  data.velo = speed.readMetersPerSecond();
  
  //read sbus
  if(sbus_rx.Read()){ //sbusの受信に成功した時に，値を更新する
    sbus_data = sbus_rx.data(); //sbusで飛んできたプロポの各チャンネルデータを読み出す
    data.ESC_u = sbus_data.ch[2] + COUNT_LOW; //ESCには3chに受信した11bitに最小値を足して送る．マニュアルでも制御でも変わらない(対気速度に対して制御してもいいかも)
    //5chのギア（投下装置）
    data.gear = sbus_data.ch[4] + COUNT_LOW;
    //制御するかどうか変更する X ? 1 : 0 は三項演算子で，この場合プロポの7ch(配列は0番からなので1ずれる)が中立より上の時1(True)となる．
    data.control_bool = (sbus_data.ch[6] > 1024) ? 1 : 0;
  }

  if(data.control_bool == 1){ //制御する場合はこの処理になる．
    
    //制御の計算を行う
    //取得したエレベータの値(2ch)を目標値に変換する（この場合0-2048を-20deg-20degに変換している）
    data.pitch_r = map(sbus_data.ch[1],0,2048,-20,20);
    data.e = data.pitch_r - data.pitch; //偏差=目標値-今の姿勢を計算
    data.eint = data.e*data.deltaT; //偏差を累積して累積偏差を計算（I制御に使用）
    data.pitch_u = -( data.Kd*data.gyroPitch + data.Kp*data.e + data.Ki*data.eint ); //制御の計算．Kd+Kp+Kiの形で制御している．この行の計算の結果は，サーボの舵角[deg]で帰ってくる
    data.pitch_u = map(data.pitch_u,-45,45,0,2048); //サーボの舵角-45,45[deg]をカウント0,2048に変換する．
    data.roll_u = sbus_data.ch[3] + COUNT_LOW; //ロール（ラダー）は制御しないので，sbusの4chそのままの値を取得．
    
    //servoにPWMとして送信
    ledcWrite(2, constrain(data.pitch_u,COUNT_LOW,COUNT_HIGH)); //constrainで舵角命令を1msから2msの範囲に制限する
    ledcWrite(3, constrain(data.roll_u,COUNT_LOW,COUNT_HIGH));
  }else{
    //制御しない場合は取得したsbusの値をそのまま送る
    data.pitch_u = sbus_data.ch[1] + COUNT_LOW;
    data.roll_u = sbus_data.ch[3] + COUNT_LOW;
    ledcWrite(2, data.pitch_u);
    ledcWrite(3, data.roll_u);
  }

  //ギアとESCは制御するかどうか問わず共通
  ledcWrite(4, data.gear);
  ledcWrite(1, data.ESC_u);

  //シリアルに出力する値
  Serial.printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f\n",millis(),data.velo,data.accX,data.accY,data.accZ,data.gyroRoll,data.gyroPitch,data.gyroYaw,data.roll,data.pitch,data.yaw,data.ESC_u,data.pitch_u,data.roll_u,data.K);

  //time 
  if(millis() - prev_time < 20) { delay(20-(millis() - prev_time)); } //今の時間millis引く前の時間で一回のループにかかる時間を計算し，その時間を使って足りなかった分をdelayで待つことでだいたい20msで制御が回ってくれるように指定
  data.deltaT = millis() - prev_time; //今の時間と一周前の時間の差を再計算
  prev_time = millis(); //今の時間を保存する

  //最後にloop関数の頭まで戻ります．

}
