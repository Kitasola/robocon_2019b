// RaspberryPiでDualShock3の信号を読取る
#ifndef ARRC_RASPI_DUALSHOCK3_HPP
#define ARRC_RASPI_DUALSHOCK3_HPP
#include <fstream>
#include <thread>

#define UPDATELOOP(c, x)                                                       \
  for ((c).update(); (x);                                                      \
       (c).update()) // コントローラcの状態をupdateしながら条件xでループ

namespace arrc_raspi {
enum ButtonsNum {
  SELECT,
  LEFT_STICK,
  RIGHT_STICK,
  START,
  UP,
  RIGHT,
  DOWN,
  LEFT,
  L2,
  R2,
  L1,
  R1,
  TRIANGLE,
  CIRCLE,
  CROSS,
  SQUARE
};
const int NumButtons = 16;
enum SticksNum { LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y, LEFT_T, RIGHT_T };
const int NumSticks = 6;
enum AxisNum { X_AXIS, Y_AXIS, Z_AXIS };
const int NumAxis = 3;
class DualShock3 {
public:
  DualShock3();
  DualShock3(bool, int timeout = 0);
  DualShock3(
      const char *, bool precision = false,
      int timeout = 0); // 必要ならファイル名を入れる デフォルトは/dev/input/js0
  void init(const char *, bool, int);
  bool connectedCheck();
  void precisionMode(bool precision = true);
  void read();
  void update(); // コントローラの状態を更新
  void yReverseSet(bool setVar = true);
  bool button(
      ButtonsNum,
      bool onlyFlag =
          false); // 指定されたボタンが押されているか返す
                  // 第2引数がtrueだとそのボタンだけが押されている場合のみtrueを返す
  bool press(ButtonsNum);
  bool release(ButtonsNum);
  int stick(SticksNum); // 指定されたスティックの状態を返す
  int acceleration(AxisNum);
  virtual ~DualShock3();

private:
  void readLoop();
  std::fstream JoyStick;
  bool loopFlag;
  bool yReverse;
  bool connectedFlag;
  bool precisionFlag;
  bool threadFlag;
  std::thread readThread;
  bool readButtonData[NumButtons];
  int readStickData[NumSticks];
  int readAxisData[NumAxis];
  bool buttonData[NumButtons];
  int stickData[NumSticks];
  int axisData[NumAxis];
  bool beforeButtonData[NumButtons];
};
} // namespace arrc_raspi
#endif
