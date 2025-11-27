/**************************************************************************
 * Lidar and Encoder data collect program (tinny ver.)
 *
 * author: Kazumichi INOUE <kazumichiinoue@mail.saitama-u.ac.jp>
 * Github: https://github.com/irlab-INOUE/coyomi2
 * date:   2025/10/4
 * update: 2025/11/27
 ****************************************************************************/
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <SDL.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <ncurses.h>
#include <atomic>
#include <mutex>
#include <thread>

using namespace std::chrono;  // seconds, milliseconds
using std::this_thread::sleep_for;
using LockGuard = std::lock_guard<std::mutex>;
std::mutex mtx;

#include "common.h"
#include "shm_board.h"
#include "yaml-cpp/yaml.h"
#include "checkDirectory.h"
#include "Config.h"
#include "time_utility.h"
#include "shared_struct.h"
#include "thread_2D_Lidar_b.h"
#include "global_variable.h"

/***************************************
 * Global variable
 ****************************************/
std::thread th_2D_Lidar_b;

// ジョイスティック
SDL_Joystick* joystick;

// 共有オブジェクト
auto log_path = std::make_shared<LOGDIR_PATH>();
auto log_data = std::make_shared<LOG_DATA>();
auto bat      = std::make_shared<BAT>();
auto loc      = std::make_shared<LOC>();
auto enc      = std::make_shared<ENC>();
auto urg2d    = std::make_shared<URG2D>();

// Oriental Motors
int fd_motor;   // FDをOrientalMotorInterface.hで使うのでinclude前に定義
#include "OrientalMotorInterface.h"
//#define DEBUG_SENDRESP

void sigcatch(int);

// The max (min) value for each axis based measurement value
struct joy_calib {
  int max;
  int min;
  int zero;

  joy_calib() {
    max = 32767;
    min = -32768;
    zero = 0;
  };

  void set_val(int val) {
    if (max < val) {
      max = val;
    }

    if (min > val) {
      min = val;
    }
  };

  void set_zero(int val) {
    zero = val;
  };
};

void read_joystick(double &v, double &w, const std::vector<joy_calib> &j_calib) {
  const int DEAD_ZONE = 8000;
  double axis0 = 0;
  double axis1 = 0;

  auto set_vw = [&v, &w](double _v, double _w) {v = _v; w = _w;};

  /* read the joystick state */
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_JOYBUTTONUP:
      case 1:
      case 2:
      case 0:
      case 3:
      default:
        set_vw(0.0, 0.0);
        break;
      case SDL_JOYBUTTONDOWN:
        switch (static_cast<int>(e.jbutton.button)) {
          case 1:
            set_vw(0.5, 0.0);
            break;
          case 2:
            set_vw(-0.5, 0.0);
            break;
          case 0:
            set_vw(0.0, 1.0);
            break;
          case 3:
            set_vw(0.0, -1.0);
            break;
          case 4:
          case 5:
            get3DLidarData.store(true);
            break;
          case 10:
            gotoEnd.store(true);
            //std::cerr << "End\n";
            set_vw(0.0, 0.0);
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            sleep_for(milliseconds(150));
            break;
          case 12:
            //std::cerr << "FREE\n";
            set_vw(0.0, 0.0);
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            sleep_for(milliseconds(100));
            isFREE.store(!(isFREE.load()));
            if (isFREE.load())
              free_motors();
            else
              turn_on_motors();
            break;
          default:
            set_vw(0.0, 0.0);
            break;
        }
    }
    if (std::abs(e.jaxis.value) > DEAD_ZONE) {
      int axis_index = static_cast<int>(e.jaxis.axis);
      switch (axis_index) {
        case 0:
          axis0 =2.0 * e.jaxis.value / (j_calib[0].max - j_calib[0].min + 10);
          break;
        case 1:
          axis1 =2.0 * e.jaxis.value / (j_calib[1].max - j_calib[1].min + 10);
          break;
        default:
          break;
      }
    }
  }
}

/***************************************
 * MAIN
 ***************************************/
int main(int argc, char *argv[]) {
  /* Ctrl+c 対応 */
  if (SIG_ERR == signal( SIGINT, sigcatch )) {
    std::printf("failed to set signal handler\n");
    exit(EXIT_FAILURE);
  }

  /* Configその他の読み込みセクション */
  // coyomi.yamlに接続する
  std::string path_to_yaml = DEFAULT_ROOT + std::string("/coyomi.yaml");
  YAML::Node coyomi_yaml = yamlRead(path_to_yaml);
  std::cerr << "coyomi.yaml is open.\n";

  // Mode selector
  std::cerr << "Hello, Coyomi2" << "\n";

  /***************************************************************************
   * LOG保管場所を作成する
   * DEFAULT_LOG_DIRの場所にcoyomi_log ディレクトリがあるかチェックし，
   * なければ作成する
   ***************************************************************************/
  // 現在日付時刻のディレクトリを作成する
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&now_c);

  checkDir(std::string(DEFAULT_LOG_DIR));
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y"); log_path->year = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%m"); log_path->mon  = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%d"); log_path->mday = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%H"); log_path->hour = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%M"); log_path->min  = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%S"); log_path->sec  = oss.str(); oss.str(""); oss.clear();
  log_path->path = std::string(DEFAULT_LOG_DIR) + "/" + log_path->year + "/" + log_path->mon + "/"
    + log_path->mday + "/" + log_path->hour + log_path->min + log_path->sec;
  checkDir(log_path->path);

  std::cerr << "path: " << log_path->path << "にログを保存します" << std::endl;

  /**************************************************************************
   * Start multi threads
   ***************************************************************************/
  th_2D_Lidar_b     = std::thread(thread_2D_Lidar_b, std::string("b"), log_path, log_data, urg2d);

  /**************************************************************************
   * Connect check & open serial port for MotorDriver
   ***************************************************************************/
  if((fd_motor = open(SERIAL_PORT_MOTOR, O_RDWR | O_NOCTTY)) == -1) {
    std::cout << "Can't open serial port" << std::endl;
    gotoEnd.store(true);
  } else {
    std::string log_text = "Get fd_motor: " + std::to_string(fd_motor);
    std::cout << log_text << std::endl;
  }

  /**************************************************************************
   * Joystick setup
   ***************************************************************************/
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0) {
    std::string log_text = "Failure SDL initialize. " + std::string(SDL_GetError());
    std::cout << log_text << std::endl;
    gotoEnd.store(true);
  }
  joystick = SDL_JoystickOpen(0);

  // calibrate axis until JS_EVENT_BUTTON pressed
  std::vector<joy_calib> j_calib(6);
  SDL_Event e;

  /**************************************************************************
   * Serial port setup for Motor Drivers
   ***************************************************************************/
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd_motor, TCSANOW, &tio);

  /**************************************************************************
   * Motor driver setup
   ***************************************************************************/
  // BLV-R Driver setup
  // ID Share Config.
  simple_send_cmd(Query_IDshare_R, sizeof(Query_IDshare_R));
  simple_send_cmd(Query_IDshare_L, sizeof(Query_IDshare_L));
  simple_send_cmd(Query_READ_R,    sizeof(Query_READ_R));
  simple_send_cmd(Query_READ_L,    sizeof(Query_READ_L));
  simple_send_cmd(Query_WRITE_R,   sizeof(Query_WRITE_R));
  simple_send_cmd(Query_WRITE_L,   sizeof(Query_WRITE_L));

  //trun on exitation on RL motor
  turn_on_motors();

  /**************************************************************************
   * initial enc setup
   ***************************************************************************/
  long long first_ts = get_current_time();
  ODOMETORY first_odo;
  read_state(first_odo, first_ts);
  sleep(1);
  enc->ts = first_ts;
  enc->x  = first_odo.rx;
  enc->y  = first_odo.ry;
  enc->a  = first_odo.ra;

  /**************************************************************************
   * Starting Main Process
   ***************************************************************************/
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  tcflush(fd_motor, TCIOFLUSH);

  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  usleep(100000);
  free_motors();

  while (isFREE.load()) {
    if (gotoEnd.load()) break;
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);

    long long ts = get_current_time();
    read_state(odo, ts);
    enc->ts = ts;
    enc->x = odo.rx;
    enc->y = odo.ry;
    enc->a = odo.ra;

    sleep_for(milliseconds(100));
  }

  while(!gotoEnd.load()) {
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);

    std::vector<LSP> lsp;
    for (int k = 0; k < urg2d->size; k++) {
      lsp.emplace_back(urg2d->r[k], urg2d->r[k]/1000.0, urg2d->ang[k], urg2d->cs[k], urg2d->sn[k]);
    }
    Pose2d estimatedPose(loc->x, loc->y, loc->a);
    v = tmp_v; w = tmp_w;

    if (isFREE.load()) {
      v = 0.0;
      w = 0.0;
    }
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    long long ts = get_current_time();
    read_state(odo, ts);

    enc->ts = ts;
    enc->x = odo.rx;
    enc->y = odo.ry;
    enc->a = odo.ra;

    std::string path = log_path->path + "/enclog";
    enc_log.open(path, std::ios_base::app);
    enc_log
      << ts << " "
      << odo.rx << " " << odo.ry << " " << odo.ra << " "
      << v << " " << w << " "
      << "end" << "\n";
    enc_log.close();
  }
  //=====<<MAIN LOOP : END>>=====

  //CLEANUP:
  // safe stop
  v = 0.0;
  w = 0.0;
  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  sleep_for(seconds(1));
  turn_off_motors();
  std::cerr << TEXT_BLUE << "Motors safe stop\n" << TEXT_COLOR_RESET;
  close(fd_motor);

  SDL_JoystickClose(joystick);
  SDL_Quit();

  enc_log.close();
  fout_urg2d.close();

  running.store(false);
  th_2D_Lidar_b.join();

  return 0;
}

void sigcatch(int sig) {
  gotoEnd.store(true);

  std::cerr << TEXT_RED;
  std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;
}
