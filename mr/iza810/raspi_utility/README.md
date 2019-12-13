# raspi_utility
* [ARRC](https://github.com/AkashiRobo)用に作成しました
* Raspbianでの使用を想定しています
* Raspberry Piでとか言ってますが他の環境でも使えるものもあります
* src/にソースファイル、include/にヘッダーファイル, test/にテストプログラムがあります
* テストプログラムはすべてmakeでコンパイルできます
* constexpr, --pthread, とかを使ってるのでc++11以上じゃないと動きません  

## 機能一覧
* [Pigpiod](#Pigpiod)
* serial
* i2c
* [Gy521](#Gy521)
* [RotaryInc](#RotaryInc)
* [MotorSerial](#MotorSerial)

************************************************************************************************

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
arrc_raspi::Pigpiod::gpio()
```
* ホスト名`arrc_raspi::PIGPIOD_HOST`, ポート`arrc_raspi::PIGPIOD_PORT`のpigpiodデーモンのハンドルを取得するコンストラクタ(デフォルトではlocalhost, 8888).
* 上記の2つの変数はオブジェクト生成前にグローバル領域に定義すること. 
(変更すると他のpigpiodを使うライブラリでも変更される)

```cpp
void arrc_raspi::Pigpiod::set(int pin, int mode, int init)
```
* ピンの設定関連
* pin番のGPIOピンが, mode=`arrc_raspi::IN`で入力用, mode=`arrc_raspi::OUT`で出力用に設定されます
* mode=`arrc_raspi::OUT`の時, init=`arrc_raspi::LOW`でLOW, `init=arrc_raspi::HIGH`でHIGHが最初に出力されます
* mode=`arrc_raspi::IN`の時, `init=arrc_raspi:PULL_UP`でプルアップ, init=`arrc_raspi::PULL_DOWN`でプルダウン, 
init=`arrc_raspi::PULL_OFF`でハイインピーダンス状態に設定されます

```cpp
void arrc_raspi::Pigpiod::write(int pin, int level)
```
* pin番のGPIOピンにlevel(0 || 1)を書き込む

```cpp
int arrc_raspi::Pigpiod::read(int pin)
```
* pin番のGPIOのレベルを返す

```cpp
int arrc_raspi::Pigpiod::checkHandle()
```
* コンストラクタで取得したハンドルを返します
* 同じプログラム内で複数のpigpiodを用いたライブラリを使う時などに使えます

```cpp
bool arrc_raspi::Pigpiod::checkInit()
```
* pigpiodの初期化に成功したかを返します

```cpp
void arrc_raspi::Pigpiod::delay(double micro_sec)
```
* micro_sec(μs)のスリープ

### テストプログラム
* 実行形式は`./test [pin] [mode] [init]`です
* 主に`arrc_raspi::Pigpiod::set(pin, mode, init)`を実行します
* 以下は`./tes`tと`set`の引数の対応表です(pinは同じ)
  > mode
  >
  > ./test | set | |
  > :---: | :---: | --- |
  > IN | arrc_raspi::IN | 入力 |
  > OUT | arrc_raspi::OUT | 出力 |
 
  > init
  >
  > ./test | set | |
  > :---: | :---: | --- |
  > 0 | arrc_raspi::LOW | LOW |
  > 1 | arrc_raspi:HIGH | HIGH |
  > UP | arrc_raspi::PULL_UP | プルアップ |
  > DOWN | arrc_raspi::PULL_DOWN | プルダウン |
  > OFF | arrc_raspi::PULL_OFF | ハイインピーダンス |
* mode=`IN`の時は`arrc_raspi::Pigpiod::read(pin)`の返り値(pin番のレベル)も表示されます

************************************************************************************************

# Gy521
* Gy521(MPU6050)というモジュールをI2C通信で扱うためのクラス
* Yaw角度の計測が出来ます
* Pigpiod, I2Cクラスを使用しています
* ファイルはGy521

## リファレンス
```cpp
arrc_raspi::Gy521::Gy521(int bit = 2, int calibration = 1000, double user_reg = 1.0)
```
* I2Cの初期化, レンジの設定, キャリブレーションをするコンストラクタ
* user_regは角速度の倍率を設定出来るので校正などに使用して下さい
* bitによって以下の表のようにレンジを変更できます

  bit | レンジ(deg/s) |
  ---: | ---: |
  0 | 250 |
  1 | 500 |
  2 | 1000 |
  3 | 2000 |
  
  なお実用的には設定したレンジの半分程度までと考えてください

```cpp
void arrc_raspi::Gy521::calibration(double calibration)
```
* ジャイロのキャリブレーションを行います
* 静止状態の値が必要なので実行中は静止させて下さい
* calibrationの値は大きいほどキャリブレーションに時間がかかりますが精度が上がります
* この関数を実行する前にupdate()は実行できません

```cpp
void arrc_raspi::Gy521::update()
```
* 前回の呼び出しからの偏差(角度)の計算をします
* スレッド立てて一定周期で角度計算する, とかはしてないのでこの関数を適当な周期で呼び出して下さい

```cpp
double arrc_raspi::Gy521::yaw
doublw arrc_raspi::Gy521::diff_yaw
```
* diff_yawは直近の`arrc_raspi::Gy521::updata()`で計算した偏差(角度)です
* yawはdiff_yawの累積, つまりYaw角度です
* `arrc_raspi::Gy521::yaw = hoge`とすることでhoge(deg)にリセット出来ます

他にキャリブレーション結果を返す関数実装予定

### テストプログラム
* 実行形式は`./test`
* 最初にキャリブレーションを行い, コンソールに結果を表示します
* コンソールに"時間(s), 角度(deg)"が出力されます
* Ctrl+Cで終了します

************************************************************************************************

# RotaryInc
* 2相式インクリメンタルロータリーエンコーダを読み取るクラス
* RasPi3B+で800Hzまで読み取り可
* Pigpiodクラスを使用しています
* コンパイル時には-lpigpiod_if2 -pthread -lrtを追加して下さい
* ファイルはrotary_inc

## リファレンス
```cpp
arrc_raspi::RotaryInc::RotaryInc(int userA, int userB, int multiplier)
```
* ピンの設定, 逓倍数の設定をするコンストラクタ
* A, B番ピンをプルアップして読み取ります
* multiplierは何逓倍かを指定します(1, 2, 4以外は1)

```cpp
int arrc_raspi::RotaryInc::get()
```
* オブジェクト生成後のパルスを返します
* 分解能×逓倍数で割ると回転数になります

### テストプログラム
* 実行形式は`./test [A] [B] [multiplier]`
* A, Bピンをプルアップし, multiplierで何逓倍か指定します
* arrc_raspi_INFOを使って, "時間(s), パルス"が出力されます
* Ctrl+Cで終了します

************************************************************************************************

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
arrc_raspi::MotorSerial::MotorSerial(int rede = 4, int timeout = 10)
```
* シリアルポートの初期化, RE・DEピンの初期化をするコンストラクタ
* redeでRE・DEピン(送受信の切り替え用ピン)が指定できる
* 送信してからtimeout(ms)の間, 受信待ちをする
* そのうちシリアルポートの指定をSerialクラスでできるようにする

```cpp
void arrc_raspi::MotorSerial::setTimeOut(int timeout)
```
* タイムアウト時間の変更をする

```cpp
short arrc_raspi::MotorSerial::sending(unsigned char id, unsigned char cmd, short data)
```
* id番のMDDに対してcmd, dataを送信します
* 返り値としてid番のMDDから返ってきたdataを返します
* 送信時に1bitごとに90μs間sleepしないと通信できません

```cpp
short arrc_raspi::MotorSerial::send(unsigned char id, unsigned char cmd, short data, bool async_flag = false)
```
* async_flag=falseなら`arrc_raspi::MotorSerial::sending`と同じ動作をします
* async_flag=trueなら非同期通信を行い, 返り値は0になります

```cpp
short arrc_raspi::MotorSerial::send(SendDataFormat send_data, bool async_flag)
```
* 引数がSendDataFormatの`arrc_raspi::MotorSerial::send()`です
* あんまり使う機会無い

```cpp
bool arrc_raspi::MotorSerial::sum_check_success_
```
* 直近の通信がsumチェックまで成功したかの結果を示します

```cpp
short arrc_raspi::MotorSerial::recent_receive_data_
```
* 直近の通信で受信したdataを示します

### テストプログラム
* 実行形式は`./test [id] [cmd] [data]`です
* `arrc_raspi::MotorSerial::sending(id, cmd, data)`を実行します
* 返り値と通信の状況が表示されます
