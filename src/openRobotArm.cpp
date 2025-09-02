#include "../../include/Robot/openRobotArm.hpp"

using namespace std;
using namespace RA;

openRobotArm::openRobotArm(int timeout)
    :fd_(-1)
{
    timeout_ = timeout;
}

openRobotArm::~openRobotArm()
{
    DisConnet();

    running_ = false;

    //thread 안전하게 종료
    if(reader_.joinable())
    {
        reader_.join();
    }

     if(parser_.joinable())
    {
        parser_.join();
    }
}

speed_t openRobotArm::getBaudrateConstant(int baudrate)
{
     switch (baudrate)
     {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 1000000: return B1000000;
        default:
            return B9600;
    }
}

bool RA::openRobotArm::write(const vector<uint8_t>& data)
{
    if(fd_ < 0) return false;

    if(data.empty()) return false;

    tcflush(fd_, TCIFLUSH);

    int n = ::write(fd_, data.data(), data.size());

    if(n != data.size()) return false;

    return true;
}

std::future<std::vector<uint8_t>> openRobotArm::AsyncWrite(const std::vector<uint8_t>& data)
{
    std::promise<std::vector<uint8_t>> promise;
    auto future = promise.get_future();
    
    if (data.empty())
    {
        promise.set_exception(std::make_exception_ptr(std::runtime_error("전송할 데이터가 비어 있음")));
        return future;
    }

    uint8_t cmd_id = data[Commandbyte];
    {
        std::lock_guard<std::mutex> lock(req_mutex_);
        pending_[cmd_id] = std::move(promise);
    }
    write(data);

    return future;
}

void openRobotArm::ReadLoop()
{
    while(running_)
    {
        int bytes_available = 0;
        ::ioctl(fd_, FIONREAD, &bytes_available); //현재 버퍼에서 읽어 올수 있는 최대 바이트

        if(bytes_available <= 0) continue;

        vector<uint8_t> buffer(bytes_available);

        int n = ::read(fd_, buffer.data(), bytes_available);
        if(n >0)
        {
            buffer.resize(n);
            std::lock_guard<std::mutex> lock(rx_Mutex_);
            rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + n);
        }

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms wait
    }

    return;
}

void openRobotArm::ParseLoop()
{
    while(running_)
    {
        std::lock_guard<std::mutex> lock(rx_Mutex_);
        while(rx_buffer_.size() >= 4)
        {
            auto it = std::search(rx_buffer_.begin(), rx_buffer_.end(),std::begin(header_), std::end(header_));
            if (it == rx_buffer_.end())
            {
                rx_buffer_.clear();
                break;
            }

            size_t pos = std::distance(rx_buffer_.begin(), it);
            if (rx_buffer_.size() < pos + 4) break; // 벡터의 범위

            uint8_t length = rx_buffer_[pos + Datalength];
            size_t packet_size = 2 + 1 + length; //header 2 , length 1

            std::vector<uint8_t> packet(rx_buffer_.begin() + pos, rx_buffer_.begin() + pos + packet_size);
            handlePacket(packet);

            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + pos + packet_size);
        }
    }
    return;
}

void openRobotArm::handlePacket(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 5) return;
    uint8_t cmd_id = packet[Commandbyte];

    std::lock_guard<std::mutex> lock(req_mutex_);
    auto it = pending_.find(cmd_id);
    if (it != pending_.end())
    {
        it->second.set_value(packet);
        pending_.erase(it);
    }

    return;
}

void openRobotArm::floatToUInt16(float data, uint8_t & high, uint8_t & low)
{
    int temp = static_cast<int>(data);

    if (temp < 0)
        temp += 65536;  // convert to unsigned (wrap around)

    high = static_cast<uint8_t>((temp >> 8) & 0xFF);
    low = static_cast<uint8_t>(temp & 0xFF);
}

float openRobotArm::uInt16ToFloat(uint8_t high, uint8_t low)
{
    int temp = low + high * 256;

    if (temp > 33000)
        temp -= 65536;

    return static_cast<float>(temp);
}

bool openRobotArm::Connet(const std::string& port, const int baudrate)
{
    baudrate_= getBaudrateConstant(baudrate);
    if (baudrate_ == 0) return false;

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
        perror("open");
        return -1;
    }
    
    memset(&tty_, 0, sizeof(tty_));

    if (tcgetattr(fd_, &tty_) != 0)
    {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty_, baudrate_);
    cfsetispeed(&tty_, baudrate_);

    tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8;
    tty_.c_iflag &= ~IGNBRK;
    tty_.c_lflag = 0;
    tty_.c_oflag = 0;
    tty_.c_cc[VMIN]  = 0;
    tty_.c_cc[VTIME] = 5;

    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_cflag |= (CLOCAL | CREAD);
    tty_.c_cflag &= ~(PARENB | PARODD);
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
    {
        perror("tcsetattr");
        return false;
    }

    running_ = true;

    reader_ = std::thread(&openRobotArm::ReadLoop,this);
    parser_ = std::thread(&openRobotArm::ParseLoop,this);

    return true;
}

bool openRobotArm::DisConnet()
{
    if(fd_ > 0)
    {
         // 버퍼 정리
        tcflush(fd_, TCIOFLUSH);

        // 포트 닫기
        close(fd_);
        std::cout << "Serial port disconnected." << std::endl;
    }

    return true;
}

bool openRobotArm::PowerOn()
{
    vector<uint8_t> cmd = { 0xFE, 0xFE, 0x02, 0x10, 0xFA };
    return write(cmd);
}

bool openRobotArm::PowerOff()
{
    vector<uint8_t> cmd = { 0xFE, 0xFE, 0x02, 0x11, 0xFA };
    return write(cmd);
}

bool openRobotArm::GetAngles(vector<float>& angles)
{
    try
    {
        vector<uint8_t> cmd = {0xFE, 0xFE, 0x02, 0x20, 0xFA};

        auto future = AsyncWrite(cmd);
        
        if(future.wait_for(std::chrono::milliseconds(timeout_)) == std::future_status::ready)
        {
            auto response = future.get();

            for(int i = 4 ; i < response.size()-1 ; i+=2)
            {
                float angle = uInt16ToFloat(response[i],response[i+1]);
                angles.push_back(angle/100.0);
            }
        }
        else
        {
            std::cout << "응답 타임아웃!" << std::endl;
            return false;
        }
    }
    catch(const std::exception& ex)
    {
        std::cerr << "예외 발생: " << ex.what() << std::endl;
        return false;
    }
    
    return true;
}

bool openRobotArm::SetAngles(const vector<float>& angles, const uint8_t speed)
{
    if (angles.size() != 6) return false;

    std::vector<uint8_t> cmd = {0xFE, 0xFE, 0X0F, 0X22};
    for (float angle : angles)
    {
        uint8_t low , high;
        floatToUInt16(angle*100,high,low); //비트 부호 반전

        cmd.push_back(high);
        cmd.push_back(low);
    }

    cmd.push_back(speed);
    cmd.push_back(0xFA);
    
    return write(cmd);
}

bool openRobotArm::SetAngle(const uint8_t joint, const float angle, const uint8_t speed)
{
    std::vector<uint8_t> cmd = {0xFE, 0xFE, 0X06, 0X21};
    cmd.push_back(joint);

    uint8_t low , high;
    floatToUInt16(angle,high,low);

    cmd.push_back(joint);
    cmd.push_back(high);
    cmd.push_back(low);

    cmd.push_back(speed);
    cmd.push_back(0xFA);
    
    return write(cmd);
}

bool openRobotArm::SetCoordi(const std::vector<float>& Coordis,const uint8_t speed, const uint8_t Mode)
{
    if(Coordis.size() != 6) return false;

    std::vector<uint8_t> cmd = {0xFE, 0xFE, 0X10, 0X25};

    for(int i = 0 ; i < 3 ; ++i)
    {
        uint8_t low , high;
        floatToUInt16(Coordis[i] *10 ,high,low);

        cmd.push_back(high);
        cmd.push_back(low);
    }

    for(int i = 3; i < Coordis.size() ; ++i)
    {
        uint8_t low , high;
        floatToUInt16(Coordis[i] *100 ,high,low);

        cmd.push_back(high);
        cmd.push_back(low);
    }

    cmd.push_back(speed);
    cmd.push_back(Mode);
    cmd.push_back(0xFA);
    
    return write(cmd);
}

bool openRobotArm::GetCoordi(std::vector<float>& coordi)
{
       try
    {
        vector<uint8_t> cmd = {0xFE, 0xFE, 0x02, 0x23, 0xFA};

        auto future = AsyncWrite(cmd);
        
        if(future.wait_for(std::chrono::milliseconds(timeout_)) == std::future_status::ready)
        {
            auto response = future.get();

            for(int i = Dataframe; i < Dataframe + 6 ; i+=2)
            {
                float Coordi = uInt16ToFloat(response[i],response[i+1]);
                coordi.push_back(Coordi/10.0);
            }

            for(int i = Dataframe + 6 ; i < response.size()-1 ; i+=2)
            {
                float angle = uInt16ToFloat(response[i],response[i+1]);
                coordi.push_back(angle/100.0);
            }
        }
        else
        {
            std::cout << "응답 타임아웃!" << std::endl;
            return false;
        }
    }
    catch(const std::exception& ex)
    {
        std::cerr << "예외 발생: " << ex.what() << std::endl;
        return false;
    }
    
    return true;
}

bool openRobotArm::SetEndCooriSys(uint8_t tool)
{
    std::vector<uint8_t> cmd = {0xFE, 0xFE, 0X03,0X89};
    cmd.push_back(tool);
    cmd.push_back(0XFA);

    return write(cmd);
}