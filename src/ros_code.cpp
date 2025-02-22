#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>
#include <cmath>
#include <mutex>
#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>

#define LEDS_PER_GROUP 6
#define DMX_START_CODE 0x00

enum ChannelIndex
{
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    WHITE = 3
};

class DMXController
{
private:
    int serialPort;
    const char *portName;
    std::vector<uint8_t> dmxData;
    const int totalGroups;
    const int totalChannels;
    const int totalLEDs;

    bool setBaudRate(int baudrate);
    int getLEDStartChannel(int groupNum, int ledNum);

public:
    DMXController(const char *port, int numGroups);
    void setLEDColor(int groupNum, int ledNum, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
    void setGroupColor(int groupNum, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
    void setAllColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
    void sendDMXFrame();
    ~DMXController();
};

class EffectController
{
private:
    DMXController &dmx;
    std::atomic<bool> running;
    std::mutex effectMutex;
    std::thread effectThread;

    void breathingEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void flashingEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void solidEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

public:
    EffectController(DMXController &dmxController);
    void startEffect(const std::string &effect, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void stopEffect();
    ~EffectController();

};

class LedModePublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ledstatuspublisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ledmodesubscriber_;
    void ledModeCallback(const std_msgs::msg::Int64::SharedPtr msg);

public:
    LedModePublisher();
};


// DMXController method implementations
bool DMXController::setBaudRate(int baudrate)
{
    struct termios2 tio;
    if (ioctl(serialPort, TCGETS2, &tio) < 0)
    {
        std::cerr << "Error getting terminal settings: " << strerror(errno) << std::endl;
        return false;
    }
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;
    tio.c_iflag &= ~(ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ICANON | ISIG);
    tio.c_cflag &= ~(PARENB | CRTSCTS);
    tio.c_cflag |= CS8 | CSTOPB | CLOCAL | CREAD;
    if (ioctl(serialPort, TCSETS2, &tio) < 0)
    {
        std::cerr << "Error setting terminal settings: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

int DMXController::getLEDStartChannel(int groupNum, int ledNum)
{
    return 1 + (groupNum * LEDS_PER_GROUP * 4) + (ledNum * 4);
}

DMXController::DMXController(const char *port, int numGroups)
    : portName(port), totalGroups(numGroups), totalChannels(numGroups * 4),
      totalLEDs(numGroups * LEDS_PER_GROUP)
{
    dmxData.resize(512, 0);
    dmxData[0] = DMX_START_CODE;
    serialPort = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serialPort < 0)
    {
        throw std::runtime_error("Failed to open serial port: " + std::string(strerror(errno)));
    }
    if (!setBaudRate(250000))
    {
        close(serialPort);
        throw std::runtime_error("Failed to set initial baud rate");
    }
    std::cout << "Serial connection established on " << port << std::endl;
}

void DMXController::sendDMXFrame()
{
    if (!setBaudRate(90000))
        throw std::runtime_error("Failed to set break baud rate");

    uint8_t breakByte = 0x00;
    if (write(serialPort, &breakByte, 1) < 0)
    {
        throw std::runtime_error("Failed to send break byte: " + std::string(strerror(errno)));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    if (!setBaudRate(250000))
        throw std::runtime_error("Failed to set data baud rate");
    std::this_thread::sleep_for(std::chrono::microseconds(12));

    ssize_t bytesWritten = write(serialPort, dmxData.data(), dmxData.size());
    if (bytesWritten != static_cast<ssize_t>(dmxData.size()))
    {
        throw std::runtime_error("Failed to write complete DMX frame");
    }
}

DMXController::~DMXController()
{
    if (serialPort >= 0)
        close(serialPort);
}

void DMXController::setLEDColor(int groupNum, int ledNum, uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    int startChannel = getLEDStartChannel(groupNum, ledNum);
    dmxData[startChannel + RED] = red;
    dmxData[startChannel + GREEN] = green;
    dmxData[startChannel + BLUE] = blue;
    dmxData[startChannel + WHITE] = white;
}

void DMXController::setGroupColor(int groupNum, uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    for (int led = 0; led < LEDS_PER_GROUP; led++)
    {
        setLEDColor(groupNum, led, red, green, blue, white);
    }
}

void DMXController::setAllColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    for (int group = 0; group < totalGroups; group++)
    {
        setGroupColor(group, red, green, blue, white);
    }
}

// EffectController method implementations
void EffectController::breathingEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    while (running)
    {
        for (int intensity = 0; intensity <= 255; intensity += 5)
        {
            {
                std::lock_guard<std::mutex> lock(effectMutex);
                dmx.setAllColor(r * intensity / 255, g * intensity / 255, b * intensity / 255, w * intensity / 255);
            }
            dmx.sendDMXFrame();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        for (int intensity = 255; intensity >= 0; intensity -= 5)
        {
            {
                std::lock_guard<std::mutex> lock(effectMutex);
                dmx.setAllColor(r * intensity / 255, g * intensity / 255, b * intensity / 255, w * intensity / 255);
            }
            dmx.sendDMXFrame();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }
}

void EffectController::flashingEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    while (running)
    {
        {
            std::lock_guard<std::mutex> lock(effectMutex);
            dmx.setAllColor(r, g, b, w);
        }
        dmx.sendDMXFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        {
            std::lock_guard<std::mutex> lock(effectMutex);
            dmx.setAllColor(0, 0, 0, 0);
        }
        dmx.sendDMXFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void EffectController::solidEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    while (running)
    {
        {
            std::lock_guard<std::mutex> lock(effectMutex);
            dmx.setAllColor(r, g, b, w);
        }
        dmx.sendDMXFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

EffectController::EffectController(DMXController &dmxController) : dmx(dmxController), running(false) {}

void EffectController::startEffect(const std::string &effect, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    stopEffect();
    running = true;
    if (effect == "breathing")
    {
        effectThread = std::thread(&EffectController::breathingEffect, this, r, g, b, w);
    }
    else if (effect == "flashing")
    {
        effectThread = std::thread(&EffectController::flashingEffect, this, r, g, b, w);
    }
    else if (effect == "solid")
    {
        effectThread = std::thread(&EffectController::solidEffect, this, r, g, b, w);
    }
    else
    {
        std::cerr << "Unknown effect: " << effect << std::endl;
    }
}

void EffectController::stopEffect()
{
    running = false;
    if (effectThread.joinable())
    {
        effectThread.join();
    }
}

EffectController::~EffectController()
{
    stopEffect();
}

LedModePublisher::LedModePublisher():Node("ros_led_pub_node")
{
ledstatuspublisher_ = create_publisher<std_msgs::msg::String>("led_status",10);
ledmodesubscriber_ = create_subscription<std_msgs::msg::Int64>("led_mode",10,
std::bind(&LedModePublisher::ledModeCallback,this ,std::placeholders::_1));


}

void LedModePublisher::ledModeCallback(const std_msgs::msg::Int64::SharedPtr msg)
{
    std::int32_t effectCode = msg->data;

    int numGroups = 9;
    DMXController dmx("/dev/ttyACM0", numGroups);
    EffectController effects(dmx);

    switch (effectCode)
    {
    case 1001:
        effects.startEffect("solid", 128, 0, 0, 0); // Solid Red
        break;
    case 1002:
        effects.startEffect("solid", 0, 0, 128, 0); // Solid Blue
        break;
    case 1003:
        effects.startEffect("solid", 0, 128, 0, 0); // Solid Green
        break;
    case 1004:
        effects.startEffect("breathing", 0, 128, 0, 0); // Breathing Green
        break;
    case 1005:
        effects.startEffect("breathing", 128, 0, 0, 0); // Breathing Red
        break;
    case 1006:
        effects.startEffect("flashing", 128, 0, 0, 0); // Flashing Red
        break;
    case 1007:
        effects.startEffect("flashing", 0, 0, 128, 0); // Flashing Blue
        break;
    case 1008:
        effects.startEffect("flashing", 0, 128, 0, 0); // Flashing Green
        break;
    case 1009:
        effects.startEffect("flashing", 0, 0, 0, 0); // No color
        break;
    default:
        std::cout << "Unknown effect code. Please try again." << std::endl;
        return;  // Exit function if the effect is unknown
    }

    // Publish status update
    auto statusmsg = std_msgs::msg::String();
    statusmsg.data = "Effect started with code " + std::to_string(effectCode);
    ledstatuspublisher_->publish(statusmsg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedModePublisher>());
    rclcpp::shutdown();
    return 0;
}