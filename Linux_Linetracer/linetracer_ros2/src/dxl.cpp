#include "linetracer_ros2/dxl.hpp"

int Dxl::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int Dxl::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

Dxl::Dxl(void)
{
    port_num = 0;
    group_num = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result
    dxl_addparam_result = false;            // AddParam result
    dxl_error = 0;                          // Dynamixel error
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
}

bool Dxl::dxl_open(void)
{    
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }

    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    return true;
}

void Dxl::dxl_close(void)
{
    // stop motor
    dxl_set_velocity(0, 0);
    
    // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port
    portHandler->closePort();
}

bool Dxl::dxl_set_velocity(int goal_velocity1, int goal_velocity2)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY);
    
    uint8_t param_goal_velocity[4];

    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity1));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity1));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity1)); 

    
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_velocity);

    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return false;
    }
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity2));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity2));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity2));

   
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_velocity);

    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return false;
    }    
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}

void Dxl::dxl_xl_open(void)
{

}

void Dxl::dxl_xl_close(void)
{

}
unsigned int Dxl::vel_convert(int speed)
{

    unsigned int temp;
    if (speed > 1023) speed = 1023;
    else if (speed < -1023) speed = -1023;
    if (speed >= 0) temp = (unsigned int)speed;
    else temp = (unsigned int)(-speed + 1023);
    return temp;
}
void Dxl::dxl_get_LR(double linear, double angular, int* speed_L, int* speed_R)
{
    double dxl_L = (linear - angular * 0.145) / (2 * 0.03); // 바퀴 반지름 : 0.03, 두 바퀴사이 거리: 0.29
    double dxl_R = (linear + angular * 0.145) / (2 * 0.03);
    *speed_L = (int)(dxl_L * 9.55 * 4.37); // (30/6.28 = 9.55)
    *speed_R = (int)(dxl_R * 9.55 * 4.37); // (30/6.28 = 9.55)
}