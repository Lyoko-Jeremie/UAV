#ifndef PACKRECEIVE_H
#define PACKRECEIVE_H

#include <QObject>
#include <QTimer>
#include <QTime>
#include <QProgressBar>
#include "datapackage.h"

enum
{
    TRANSFER_ERROR    = 0xFFFFFFFE,
    TRANSFER_COMPLETE = 0xFFFFFFFF,
};

enum
{
    TYPE_PHOTO      = 0, //照片数据
    TYPE_PYTHON     = 1, //python程序
    TYPE_ERROR      = 2, //离线程序错误信息
};

#define UNITS_LEN 26
#define TIME_OUT 5000

class PackRreceive : public QObject
{
    Q_OBJECT

public:
    PackRreceive();

    u8 unitsMark[800];//320*240*2/26/8
    u8 packBuff[153600];//320*240*2 (最大可以传输150K)
    u32 packLen;

public slots:

    void readInit(u32 buffLen);
    void readStop(void);
    void readStart(start_t);
    void readPacks(pack_t);
    void readUnits(u32 unitsCnt,u8 *unitsBuff);
    void readTimeOut(void);

//    void writeInit(QByteArray pack);
//    void writeStop(void);
//    void writeStart(start_t);
//    void writePacks(pack_t);

private:

    u32 markLen;
    u32 lostMark;

    bool reading = false;

    void getLostMark(void);
    void setMark(u32 index);
    u8 getMark(u32 index);
    void showProgress(int progress);

    QTimer timeout;
    start_t start = {0,0xFFFF,0,0};

signals:

    void signalLostMark(lost_t);//丢失重传
    void signalSucceed(start_t); //已完成信号
    void signalProgress(int,QString);
};

class PackTransmit : public QObject
{
    Q_OBJECT

public:

    PackTransmit();

    void write(u8 id,u8 type,QByteArray pack);
    void showProgress(int progress);

public slots:
    void run(void);
    void stop(void);
    void readLostMark(lost_t);
    void sendPack(u32 mark);

private:

    QTimer timer;
    QTime updateTime;
    QByteArray packArray;
    start_t packStart = {0,0,0,0};

    u32 markLen = 0;
    u32 intunitsLen = 0;
    u32 lostMark = 0;
    u32 sendMark = 0;
    u32 updateMs = 0;

    bool writing = false;

signals:

    void signalPackStart(start_t);
    void signalSendPack(pack_t);
    void signalProgress(int,QString);
};

#endif // PACKRECEIVE_H
