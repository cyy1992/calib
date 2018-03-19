#ifndef DATAMATRIXDETECTOR_H
#define DATAMATRIXDETECTOR_H

#include <thread>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <opencv2/opencv.hpp>

class DataMatrixDetector
{
public:
  enum ErrorCode
  {
    SUCCEED,
    COARSELOCFAILED,
    DETECTING,
    DETECTED
  };

  enum DataMatrixType
  {
    DmtxSymbolRectAuto = -3,
    DmtxSymbolSquareAuto = -2,
    DmtxSymbolShapeAuto = -1,
    DmtxSymbol10x10 = 0,
    DmtxSymbol12x12,
    DmtxSymbol14x14,
    DmtxSymbol16x16,
    DmtxSymbol18x18,
    DmtxSymbol20x20,
    DmtxSymbol22x22,
    DmtxSymbol24x24,
    DmtxSymbol26x26,
    DmtxSymbol32x32,
    DmtxSymbol36x36,
    DmtxSymbol40x40,
    DmtxSymbol44x44,
    DmtxSymbol48x48,
    DmtxSymbol52x52,
    DmtxSymbol64x64,
    DmtxSymbol72x72,
    DmtxSymbol80x80,
    DmtxSymbol88x88,
    DmtxSymbol96x96,
    DmtxSymbol104x104,
    DmtxSymbol120x120,
    DmtxSymbol132x132,
    DmtxSymbol144x144,
    DmtxSymbol8x18,
    DmtxSymbol8x32,
    DmtxSymbol12x26,
    DmtxSymbol12x36,
    DmtxSymbol16x36,
    DmtxSymbol16x48
  };

  struct DetectParams
  {
    int imgWidth;   //图像宽度
    int imgHeight;  //图像高度
    int edgeMin;    //二维码边缘长度极小值（ICARRIER下方42；IAGV
                    //17；ICARRIER上方32）
    int edgeMax;    //二维码边缘长度极大值（IAGV 200；ICARRIER 52）
    int edgeThresh; //种子点边缘梯度阈值（77）
    int edgeStopThresh; //边缘终点梯度阈值（50）
    int solidContiAngRng; //连续运动时，实线边的霍夫变换的角度正负搜索范围（15）
    int dashedContiAngRng; //连续运动时，虚线边的霍夫变换的角度正负搜索范围（2）
    DataMatrixType type; //二维码类型（IAGV DmtxSymbolSquareAuto；ICARRIER
                         //DmtxSymbol10x10）
    int isCoarse2Fine; //检测输入是否为粗定位后的二维码图像（0 不是，非0 是）
  };

  struct CoarseLocParams
  {
    int leastGridCnt;   //最小连通区域个数（2）
    int gridSize;       //网格尺寸（30）
    int locDriftThresh; //连通网格区域的中心偏移阈值（60）

    float ratio_k;
    int min_min_k;
    int min_max_k;
    int max_max_k;
  };

  struct DataMatrixInfo
  {
    double x;         //二维码角点图像坐标
    double y;         //二维码角点图像坐标
    double angle;     //二维码底边与x轴的夹角（弧度）
    char message[80]; //二维码信息
  };

  /* params1	二维码检测参数
   * params2	粗定位参数*/
  DataMatrixDetector(const DetectParams& params1,
                     const CoarseLocParams& params2 = CoarseLocParams());
  ~DataMatrixDetector();

  /* 粗定位
   * srcImg		IN	输入原始图像
   * centerLoc	OUT	二维码中心点像素坐标*/
  ErrorCode coarseLoc(const cv::Mat& srcImg, cv::Point2d* centerLoc);

  /* 导入待检测数据
   * srcImg			IN	原始图像
   * continuousAction	IN	原始图像*/
  bool detect(const cv::Mat& srcImg, bool continuousAction = true);

  /* 停止检测*/
  void abort();

  /* 提取最新检测结果
   * dmInfo	OUT	检测结果
   * 返回值		检测是否已结束*/
  ErrorCode getDetectInfo(std::vector<DataMatrixInfo>& dmInfo);

  /* 参数计算，用于在视野大小、二维码尺寸等信息产生变化时
  * objecject_size	实际视野范围，单位为mm
  * dm_obj_size		实际二维码尺寸，单位为mm
  * label_obj_size	实际标签尺寸，单位为mm
  * best_param		最佳分辨率，单位为像素/mm*/
  void calc_param(const double object_size, const double dm_obj_size,
                  const double label_obj_size, const double best_param);

  static void notify_all();

private:
  enum DetectState
  {
    IDLE,
    READY,
    BUSY,
    ABORT = -1
  };

  void detectLoop();

  DetectParams dp;
  CoarseLocParams clp;
  cv::Mat procImg;
  void* cacheBuf;
  void* outlineBuf;
  int bufLen;
  int outlineBufLen;
  int16_t lastBottomAng;
  DetectState ds;
  bool keepRunning;
  std::vector<DataMatrixInfo> dmResults;

  std::thread detectThread;
  std::mutex detectStateMtx;
  std::mutex getResultsMtx;
  std::condition_variable detectProcFinishedCS;
  std::condition_variable detectDataReadyCS;

  static std::mutex sharedMtx;
  static std::condition_variable sharedCS;
};

#endif
