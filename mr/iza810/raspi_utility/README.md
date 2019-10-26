# ros_raspi_utility
* [ARRC](https://github.com/AkashiRobo)用に作成しました
* Ubuntu 16.04 LTS, Kinetic での使用を想定しています
* Raspberry Piでとか言ってますが一部は他の環境でも使えるものもあります
* src/にソースファイル、include/にヘッダーファイル, test/にテストプログラムがあります
* テストプログラムはすべてmakeでコンパイルできます
* constexpr, --pthread, とかを使ってるのでc++11以上じゃないと動きません  

## 機能一覧
* [Pigpiod](#Pigpiod)
* serial
* i2c
* [GY521](#GY521)
* [RotaryInc](#RotaryInc)
* [MotorSerial](#MotorSerial)

*************************************************************************************************

# Pigpiod
* [pigpio](http://abyz.me.uk/rpi/pigpio/index.html)の[pigpiod](http://abyz.me.uk/rpi/pigpio/pigpiod.html)
というデーモンを用いてGPIOを操作するためのクラス
* ~~1つのプログラムにつき1つオブジェクトで使って下さい~~ シングルトン使ってるので大丈夫です
* pigpiodデーモンは予め起動して下さい
  > sudo pigpiodで起動できる. 自動起動は/etc/rc.localに追記するのがおすすめ
* コンパイル時には-lpigpiod_if2 -pthread -lrtを追加して下さい
* ファイルはpigpiod

## リファレンス
```cpp
ros::Pigpiod::gpio()
```
* ホスト名`ros::PIGPIOD_HOST`, ポート`ros::PIGPIOD_PORT`のpigpiodデーモンのハンドルを取得するコンストラクタ
* デフォルトではlocalhost, 8888. 他のRasPiのGPIO操作などする時はオブジェクト生成前に変更すること 
(変更すると他のpigpiodを使うライブラリでも変更される)

```cpp
void ros::Pigpiod::gpio().set(int pin, int mode, int init)
```
* ピンの設定関連
* pin番のGPIOピンが, mode=`ros::IN`で入力用, mode=`ros::OUT`で出力用に設定されます
* mode=`ros::OUT`の時, init=`ros::LOW`でLOW, `init=ros::HIGH`でHIGHが最初に出力されます
* mode=`ros::IN`の時, `init=ros:PULL_UP`でプルアップ, init=`ros::PULL_DOWN`でプルダウン, 
init=`ros::PULL_OFF`でハイインピーダンス状態に設定されます

```cpp
void ros::Pigpiod::gpio().write(int pin, int level)
```
* pin番のGPIOピンにlevel(0 || 1)を書き込む

```cpp
int ros::Pigpiod::gpio().read(int pin)
```
* pin番のGPIOのレベルを返す

```cpp
int ros::Pigpiod::gpio().checkHandle()
```
* コンストラクタで取得したハンドルを返します
* 同じプログラム内で複数のpigpiodを用いたライブラリを使う時などに使えます

```cpp
bool ros::Pigpiod::gpio().checkInit()
```
* pigpiodの初期化に成功したかを返します

```cpp
void ros::Pigpiod::gpio().delay(double micro_sec)
```
* micro_sec(μs)のスリープ

### テストプログラム
* 実行形式は`./test [pin] [mode] [init]`です
* 主に`ros::Pigpiod::set(pin, mode, init)`を実行します
* 以下は`./tes`tと`set`の引数の対応表です(pinは同じ)
  > mode
  >
  > ./test | set | |
  > :---: | :---: | --- |
  > IN | ros::IN | 入力 |
  > OUT | ros::OUT | 出力 |
 
  > init
  >
  > ./test | set | |
  > :---: | :---: | --- |
  > 0 | ros::LOW | LOW |
  > 1 | ros:HIGH | HIGH |
  > UP | ros::PULL_UP | プルアップ |
  > DOWN | ros::PULL_DOWN | プルダウン |
  > OFF | ros::PULL_OFF | ハイインピーダンス |
* mode=`IN`の時は`ros::Pigpiod::read(pin)`の返り値(pin番のレベル)も表示されます

*************************************************************************************************

# GY521
* GY521(MPU6050)というモジュールをI2C通信で扱うためのクラス
* Yaw角度の計測が出来ます
* Pigpiod, I2Cクラスを使用しています
* ファイルはGY521

## リファレンス
```cpp
ros::GY521::GY521(int bit = 2, int calibration = 1000, double user_reg = 1.0)
```
* I2Cの初期化, レンジの設定, キャリブレーションをするコンストラクタ
* キャリブレーションには静止状態の値が必要なので実行中は静止させて下さい
* calibrationは大きいほどキャリブレーションに時間がかかりますが精度が上がります
* user_regは角速度の倍率を設定出来るので校正などに使用して下さい
* bitによって以下の表のようにレンジを変更できます

  bit | レンジ(deg/s) |
  ---: | ---: |
  0 | 250 |
  1 | 500 |
  2 | 1000 |
  3 | 2000 |
  
  なお実用的な設定したレンジの半分程度までと考えてください

```cpp
void ros::GY521::update()
```
* 前回の呼び出しからの偏差(角度)の計算をします
* スレッド立てて一定周期で角度計算する, とかはしてないのでこの関数を適当な周期で呼び出して下さい

```cpp
double ros::GY521::yaw_
doublw ros::GY521::diff_yaw_
```
* diff_yaw_は直近の`ros::GY521::updata()`で計算した偏差(角度)です
* yaw_はdiff_yaw_の累積, つまりYaw角度です
* `ros::GY521::yaw_ = hoge`とすることでhoge(deg)にリセット出来ます

```cpp
void ros::GY521::start(double start = 0)
```
* `ros::GY521::updata()`の初回呼び出し前に1度だけ実行して下さい
* `ros::GY521::yaw_`がstart(deg)で初期化されます

他にキャリブレーション結果を返す関数実装予定

### テストプログラム
* 実行形式は`./test`
* 最初にキャリブレーションを行い, ROS_INFOで結果を表示します
* ROS_INFOを使って, "時間(s), 角度(deg)"が出力されます
* Ctrl+Cで終了します

*************************************************************************************************

# RotaryInc
* 2相式インクリメンタルロータリーエンコーダを読み取るクラス
* RasPi3B+で800Hzまで読み取り可
* Pigpiodクラスを使用しています
* コンパイル時には-lpigpiod_if2 -pthread -lrtを追加して下さい
* ファイルはrotary_inc

## リファレンス
```cpp
ros::RotaryInc::RotaryInc(int userA, int userB, int multiplier)
```
* ピンの設定, 逓倍数の設定をするコンストラクタ
* A, B番ピンをプルアップして読み取ります
* multiplierは何逓倍かを指定します(1, 2, 4以外は1)

```cpp
int ros::RotaryInc::get()
```
* オブジェクト生成後のパルスを返します
* 分解能×逓倍数で割ると回転数になります

### テストプログラム
* 実行形式は`./test [A] [B] [multiplier]`
* A, Bピンをプルアップし, multiplierで何逓倍か指定します
* ROS_INFOを使って, "時間(s), パルス"が出力されます
* Ctrl+Cで終了します

*************************************************************************************************

# MotorSerial
* ARRCのMDDと通信するためのクラス
* RasPiがマスター, Arduino, Nucleo(L432KC)がスレーブ
* UARTをRS485(半2重)に変換し通信する
* Arduinoには[これ](https://github.com/Kitasola/ScrpMotor2018)を書き込む
  > その内STM32(Nucleo)をスレーブにするプログラムも実装予定
* [元になったプログラム](https://github.com/owl8/RasPiMS)を作成された偉大なる大先輩に圧倒的感謝
* Pigpiod, Serialクラスを使用
* ファイルはmotor_serial

## リファレンス
```cpp
ros::MotorSerial::MotorSerial(int rede = 4, int timeout = 10)
```
* シリアルポートの初期化, RE・DEピンの初期化をするコンストラクタ
* redeでRE・DEピン(送受信の切り替え用ピン)が指定できる
* 送信してからtimeout(ms)の間, 受信待ちをする
* そのうちシリアルポートの指定をSerialクラスでできるようにする

```cpp
void ros::MotorSerial::setTimeOut(int timeout)
```
* タイムアウト時間の変更をする

```cpp
short ros::MotorSerial::sending(unsigned char id, unsigned char cmd, short data)
```
* id番のMDDに対してcmd, dataを送信します
* 返り値としてid番のMDDから返ってきたdataを返します
* 送信時に1bitごとに90μs間sleepしないと通信できません

```cpp
short ros::MotorSerial::send(unsigned char id, unsigned char cmd, short data, bool async_flag = false)
```
* async_flag=falseなら`ros::MotorSerial::sending`と同じ動作をします
* async_flag=trueなら非同期通信を行い, 返り値は0になります

```cpp
short ros::MotorSerial::send(SendDataFormat send_data, bool async_flag)
```
* 引数がSendDataFormatの`ros::MotorSerial::send()`です
* あんまり使う機会無い

```cpp
bool ros::MotorSerial::sum_check_success_
```
* 直近の通信がsumチェックまで成功したかの結果を示します

```cpp
short ros::MotorSerial::recent_receive_data_
```
* 直近の通信で受信したdataを示します

### テストプログラム
* 実行形式は`./test [id] [cmd] [data]`です
* `ros::MotorSerial::sending(id, cmd, data)`を実行します
* 返り値と通信の状況が表示されます
