#include <QDebug>
#include "packtransfer.h"

PackRreceive::PackRreceive()
{
    connect(&timeout,SIGNAL(timeout()),this,SLOT(readTimeOut()));//定时器事件
}

void PackRreceive::readInit(u32 buffLen)
{
    lostMark = 0;
    packLen  = buffLen; //总数据量
    markLen  = buffLen/UNITS_LEN; //总包量
    if(buffLen-markLen*UNITS_LEN) markLen++;
    for(int i=0;i<800;i++) unitsMark[i] = 0;
}

void PackRreceive::readUnits(u32 index,u8 *buff)
{
    if( index<markLen && !getMark(index) )
    {
        u8 len = UNITS_LEN;
        u32 remain = packLen - index*UNITS_LEN;
        if(len>remain) len = remain;

        memcpy(packBuff+index*UNITS_LEN,buff,len);

        setMark(index);
    }

    getLostMark();
}

void PackRreceive::setMark(u32 index)
{
    u32 cnt = index/8;
    unitsMark[cnt]|=1<<(index-cnt*8);
}

u8 PackRreceive::getMark(u32 index)
{
    u32 cnt = index/8;
    return unitsMark[cnt]&(1<<(index-cnt*8));
}

void PackRreceive::getLostMark(void)
{
    for(;lostMark<markLen;lostMark++) if(!getMark(lostMark)) break;
    if(lostMark==markLen) lostMark = TRANSFER_COMPLETE;//接收完整
}

void PackRreceive::readStop(void)
{
    timeout.stop();
    start.count = 0xFFFF;
    reading = false;
}

void PackRreceive::readTimeOut(void)
{
    readStop();
    emit showProgress(-1);
}

void PackRreceive::readStart(start_t read)
{
    //判断新的数据包
    if(!reading)
    {
        //空闲状态可以接收任一设备的数据
        if( start.id!=read.id || start.count!=read.count )
        {
            start = read;
            reading = true;
            timeout.start(TIME_OUT);
            readInit(start.size);
            emit showProgress(0);
        }
    }
    else
    {
        //正在读取中可以被当前设备打断重传
        if( start.id==read.id && start.count!=read.count )
        {
            start = read;
            reading = true;
            timeout.start(TIME_OUT);
            readInit(start.size);
            emit showProgress(0);
        }
    }

    //LostMark从0开始
    if( reading && start.count==read.count && start.id==read.id )
    {
        lost_t lost = {start.id,start.count,0};
        emit signalLostMark(lost);
    }
}

void PackRreceive::readPacks(pack_t pack)
{
    readUnits(pack.units,pack.buff);

    if( lostMark!=(pack.units+1) )
    {
        lost_t  lost={start.id,start.count,lostMark};
        emit signalLostMark(lost);//请求重传
    }

    if(lostMark==TRANSFER_COMPLETE)
    {
        if(reading)
        {
            readStop();
            emit signalSucceed(start);//已完成信号
            emit showProgress(100);
        }
    }
    else
    {
        timeout.start(TIME_OUT);//重新计时，连续5秒收不到信号停止接收
        emit showProgress(int(lostMark*100/markLen));
    }
}

void PackRreceive::showProgress(int progress)
{
    QString string = "数据接收: ";

    if(progress<0) string += "FAIL";
    else if(progress==100) string += "OK";
    else string += QString::number(progress)+" %";

    emit signalProgress(start.type,string);
}

//**********以下是发送部分*************//

PackTransmit::PackTransmit()
{
    qRegisterMetaType<start_t>("start_t");
    connect(&timer,SIGNAL(timeout()),this,SLOT(run()));//定时器事件
}

void PackTransmit::write(u8 id,u8 type,QByteArray pack)
{
    if(pack.length()==0) return;

    if(!writing)
    {
        writing         = true;
        packArray       = pack;
        packStart.id    = id;
        packStart.type  = type;
        packStart.size  = packArray.length();
        packStart.count = packStart.count + 1; if(packStart.count>=0xFFFF) packStart.count = 0;
        markLen         = int(packStart.size/26); if(packStart.size-markLen*26) markLen++;
        lostMark        = TRANSFER_ERROR;
        sendMark        = 0;
        updateTime      = QTime::currentTime();
        timer.start(2);
        emit showProgress(0);
    }
}

void PackTransmit::readLostMark(lost_t lost)
{
    static u32 lostTemp;
    static QTime lostTime;

    if( lost.id==packStart.id && lost.count==packStart.count)
    {
        lostMark   = lost.mark;
        updateTime = QTime::currentTime();

        if( sendMark<lostMark )
        {
            sendMark = lostMark;

        }
        else
        {
            if(lostTemp!=lostMark)
            {
                lostTime = QTime::currentTime();
                lostTemp = lostMark;
            }
            else
            {
                if(lostTime.msecsTo(QTime::currentTime())>20)
                {
                    lostTime = QTime::currentTime();
                    sendMark = lostMark;
                }
            }
        }
    }
}

void PackTransmit::sendPack(u32 mark)
{
    u8 unitsLen = UNITS_LEN;
    int remain = packStart.size - mark*26;
    if(unitsLen> remain) unitsLen = remain;

    pack_t pack;
    pack.units = mark;
    for(u8 i=0;i<unitsLen;i++)
    {
        pack.buff[i] = packArray[mark*26+i];
    }

    emit signalSendPack(pack);
}

void PackTransmit::stop(void)
{
    writing = false;
    packArray.clear();
    timer.stop();
}

void PackTransmit::run(void)
{
    switch(lostMark)
    {
        //发送成功
        case TRANSFER_COMPLETE:
        {
            stop(); emit showProgress(100);
        }
        break;

        //发送包头
        case TRANSFER_ERROR:
        {
            if( updateTime.msecsTo(QTime::currentTime())>1000 )
            {
                stop(); emit showProgress(-1);
            }
            else
            {
                emit signalPackStart(packStart);
            }
        }
        break;

        //发送数据
        default:
        {
            if( updateTime.msecsTo(QTime::currentTime())>TIME_OUT )
            {
                stop(); emit showProgress(-1);
            }
            else
            {
                emit showProgress(int(lostMark*100/markLen));
                sendPack(sendMark);
                if(++sendMark>=markLen) sendMark = lostMark;
            }
        }
        break;
    }
}

void PackTransmit::showProgress(int progress)
{
    QString string = "数据发送: ";

    if(progress<0) string += "FAIL";
    else if(progress==100) string += "OK";
    else string += QString::number(progress)+" %";

    emit signalProgress(packStart.type,string);
}
