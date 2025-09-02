#include <iostream>
#include <fcntl.h>      // open
#include <termios.h>    // termios, tcgetattr, tcsetattr
#include <unistd.h>     // read, write, close
#include <cstring>      // memset
#include <cstdint>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <mutex>
#include <memory>
#include <future>
#include <sys/ioctl.h>
#include <iomanip>
#include <functional>


namespace RA
{
    enum hadder : int { Headerbyte0 = 0, Headerbyte1, Datalength, Commandbyte, Dataframe};
    enum Axis : int { X = 0, Y, Z, RX, RY, RZ };
    enum Joint : int { J1 = 0, J2, J3, J4, J5, J6 };

    class openRobotArm
    {
    private:
        int timeout_;                                //Timeout     
        int fd_;                                     //filedescriptor
        speed_t baudrate_;
        termios tty_;

        bool connected_;

        std::thread reader_;
        std::thread parser_;
        bool running_;

        std::mutex rx_Mutex_;

        std::vector<uint8_t> rx_buffer_;
        const uint8_t header_[2] = {0xFE, 0xFE};

        std::mutex req_mutex_;
        std::map<uint8_t, std::promise<std::vector<uint8_t>>> pending_;

        //외부에서 호출 하면 안되는 함수들
        speed_t getBaudrateConstant(int baudrate);

        bool write(const std::vector<uint8_t>& data);
        std::future<std::vector<uint8_t>> AsyncWrite(const std::vector<uint8_t>& data);

        void ReadLoop();
        void ParseLoop();
        void handlePacket(const std::vector<uint8_t>& packet);

        void floatToUInt16(float data, uint8_t & low, uint8_t & high);
        float uInt16ToFloat(uint8_t low, uint8_t high);

    public:
        using s_ptr = std::shared_ptr<openRobotArm>;
        
        openRobotArm(int timeout = 500);
        ~openRobotArm();

        bool Connet(const std::string& port, const int baudrate = 9600);
        bool DisConnet();
        bool isConnected() const;

        bool PowerOn();
        bool PowerOff();

        bool GetAngles(std::vector<float>& angles);
        bool SetAngles(const std::vector<float>& angles,const uint8_t speed);
        bool SetAngle(const uint8_t joint, const float angle, const uint8_t speed);

        bool SetCoordi(const std::vector<float>& Coordis,const uint8_t speed, const uint8_t Mode = 1);
        bool GetCoordi(std::vector<float>& coordi);
        bool SetEndCooriSys(uint8_t tool);

        // bool isMoving();

        // bool GetCoords();
        // bool SetCoords();

        // bool SetLEDrgb(int r , int g , int b);

        // bool SetGrippeerState();
        // bool GetGripperValue();

        // bool SetSpeed();
        // bool GetSpeed();
    };
}
