#ifndef LINGAO_DATA_STREAM_H
#define LINGAO_DATA_STREAM_H

#include <vector>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include "lingao_base/data_format.hpp"
#include "lingao_base/Serial_Async.hpp"

typedef std::vector<unsigned char> vecBuff;

enum Data_Receive_Enum
{
    DATA_RECEIVE_FIND_HEAD1,
    DATA_RECEIVE_FIND_HEAD2,
    DATA_RECEIVE_MSG_ID,
    DATA_RECEIVE_LENGHT,
    DATA_RECEIVE_DATA,
    DATA_RECEIVE_CHECK
};

class Data_Stream
{
public:
    Data_Stream(Transmission *_trans);
    bool version_detection(void);

    void rxdata_parsing(vecBuff newdata);
    bool get_Message(Message_Id_Enum msgId, int timeoutMs = 50);
    void update_liner_speed(Data_Format_Liner linertx);

    bool msg_Transmit(Message_Id_Enum msgId);
    bool msg_Transmit(MessageFormat_st msgf);
    bool msg_Transmit(Message_Id_Enum msgId, unsigned char *data, int size);

private:
    Data_Receive_Enum Receive_State;
    MessageHead_st rx_Head;

    vecBuff read_buffer;
    Message_Id_Enum getMessageBlockingID;

    // boost::asio::io_service io_sev_;
    // boost::asio::deadline_timer timeout;
    std::mutex getData_mutex_;
    std::mutex receive_mutex_;

    Data_Format_Liner rxData_liner;
    Data_Format_IMU rxDdata_imu;
    Data_Format_BAT rxData_battery;
    Data_Format_RC rxData_rc;

    std::mutex m;
    std::condition_variable cv;

    bool search_Head(unsigned char data);
    void msg_ReceiveCallBack(size_t len);
    void data_undecode(MessageFormat_st msgData);

    Transmission *trans;

    Data_Format_VER version;
    bool verError;

public:
    bool onBoardImuAvailable()
    {
        if (version.protoVer >= LA_PROTO_VER_0220)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool rcAvailable()
    {
        if (version.protoVer >= LA_PROTO_VER_0310)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    Data_Format_Liner get_data_liner()
    {
        std::lock_guard<std::mutex> lock(getData_mutex_);
        return rxData_liner;
    }

    Data_Format_IMU get_data_imu()
    {
        std::lock_guard<std::mutex> lock(getData_mutex_);
        return rxDdata_imu;
    }

    Data_Format_BAT get_data_battery()
    {
        std::lock_guard<std::mutex> lock(getData_mutex_);
        return rxData_battery;
    }

    Data_Format_VER get_data_version()
    {
        std::lock_guard<std::mutex> lock(getData_mutex_);
        return version;
    }

    Data_Format_RC get_data_rc()
    {
        std::lock_guard<std::mutex> lock(getData_mutex_);
        return rxData_rc;
    }
};

#endif // LINGAO_DATA_STREAM_H
