#include "dector.h"
#include <thread>
#include "socketclient.h"
#include <unistd.h>
#include <mutex>

//串口相关的头文件
#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include <thread>

//宏定义
#define FALSE  -1
#define TRUE   0
#define PATH "/dev/ttyUSB0"

Dector dector;
int fd;
int clnt_sock;
mutex sLock;
float err_last = 0;
int D_value = 0;
int MAX_D_VALUE = 40;
float integral = 0;
float Kp = 0.12,Ki = 0,Kd = 0;
int speed = 50;
volatile bool stopPid = false;
double lastTime = 0;
float position_y;
float RD;
int delay_time;
//int car_speed;
double last_position_err = 0;
int pid_count;
bool finishedTurn = false;
bool turned = false;

void MotroCarControl();
void PIDControl();
void setTimer();
void readFromExpress();
void turn(int time, int type);
void adjustDirection();
int UART_Init(char* path, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART_Recv(int fd, char *rcv_buf,int data_len);
int UART_Send(int fd, char *send_buf,int data_len);

int main(int argc, char *argv[]){
    printf("start1!\n");
    fd = UART_Init(PATH, 115200, 0, 8, 1, 'N');

//    char send_buf[11]="z 10 -10;\r";
//    if(UART_Send(fd,send_buf,10) > 0){
//        printf("send data successful\n");
//    } else {
//        printf("send data failed!\n");
//    }
//    while(1){
//        sleep(1000);
//    }
//    cout << "end" << endl;
//    return 0;


//    struct sockaddr_in serv_addr;

//    clnt_sock = socket(AF_INET, SOCK_STREAM, 0);
//    if (clnt_sock < 0) {
//        printf("sock() error\n");
//        exit(1);
//    }

//    memset(&serv_addr, 0, sizeof(serv_addr));
//    serv_addr.sin_family = AF_INET;
//    serv_addr.sin_addr.s_addr = inet_addr("192.168.1.105");
//    serv_addr.sin_port = htons(8899);

//    if (connect(clnt_sock,
//            (struct sockaddr*)&serv_addr,
//            sizeof(serv_addr)) == -1) {
//        printf("connect() error\n");
//        exit(1);
//    }
//    char * initMsg = "h1e";
//    write(clnt_sock, initMsg, strlen(initMsg));
//    usleep(500000);
//    char * stateMsg = "s1e";
//    write(clnt_sock, stateMsg, strlen(stateMsg));


    lastTime = clock();
    thread th1(MotroCarControl);
    thread th2(setTimer);
//    thread commThread(readFromExpress);

//    dector.videoTest("/home/ubuntu/Documents/images/1.AVI", clnt_sock);//
    dector.cameraTest(clnt_sock);

//    for(int i = 1; i <= 15; i++) {
//        dector.imageTest("/home/ubuntu/Documents/images/" + to_string(i) + ".JPG");
//    }
//    close(clnt_sock);
    th1.join();
    return 0;
}

void readFromExpress(){
    char buf[1];
    char data[50];
    int index = 0;
    cout << "reading" << endl;
    while (1) {
        memset(buf, 0, sizeof(buf));
        ssize_t size = read(clnt_sock, buf, sizeof(buf));

        if (size >= 0) {
            data[index++] = buf[0];
            if(data[index - 1] == 'e') {
//                cout << "rec from express:" << buf[0] << endl;
                for(int i = 0; i < index; i++) {
                    if(data[i] == 'm') {
                        if(data[i + 1] == '0' && data[i + 2] == '2') {
                dector.commandStop = true;
                dector.stopDecode = true;
                            cout << "comm:stop" << endl;
                            char * stateMsg = "s2e";
                            write(clnt_sock, stateMsg, strlen(stateMsg));
                            break;
                        }
                        if(data[i + 1] == '0' && data[i + 2] == '1') {
                            cout << "comm:farword" << endl;
                            stopPid = false;
                dector.stopDecode = false;
                            char * stateMsg = "s1e";
                            write(clnt_sock, stateMsg, strlen(stateMsg));
                            break;
                        }
                    } else if(data[i] == 'r') {
                        dector.nodeIndex = 0;
                        int count_l = 0;
                        for(int j = i + 1; j < index; j++) {
                            if(data[j] == 'l' || data[j] == 'e') {
                                for(int m = j - count_l; m < j; m++) {
                                    if(data[m] == 'i') {
                                        int value = 0, comm = 0;
                                        for(int n = j -count_l; n < m; n++) {
                                            value += ((data[n] - 48)*pow(10, m - n - 1));
                                        }
                                        if(data[j] == 'e') {
                                            cout << "stop:" << value << data[++m] << data[++m];
                                            dector.stopNum = value;
                                            dector.routeNodes[dector.nodeIndex] = value;
                                            dector.command[dector.nodeIndex] = 5;
                                            dector.nodeIndex++;
                                        } else if(data[j] == 'l') {
                                            comm = data[m + 2] - 48;
                                            dector.routeNodes[dector.nodeIndex] = value;
                                            dector.command[dector.nodeIndex] = comm;
                                            dector.nodeIndex++;
                                        }
                                    }
                                }
                                count_l = 0;
                                cout << endl;
                            } else {
                                count_l++;
                            }
                        }
                        for(int i = 0; i < dector.nodeIndex; i++) {
                            cout << dector.routeNodes[i] << "//" << dector.command[i] << endl;
                        }
                        dector.nodeIndex = 0;
            dector.decode_value = 0;
                        break;
                    }
                }
                index = 0;
                memset(data, 0, sizeof(data));
            }
        } else {
            printf("read() error\n");
            break;
        }
    }
}

void MotroCarControl(){
    sleep(1);
    while(true){

        if(dector.readyToTurn){
         cout <<"the readToTurn is true"<<endl;
            if(dector.centre_y > 10){
                position_y = (360 - dector.centre_y)*2/1000000;
                RD =0.29*(1.052 + 368.48*position_y)/(2.822 -622*position_y);
                delay_time =1000000*(RD + 0.40)/0.489;
                cout <<"the centre_y > 10 "<<endl;
                turn(delay_time, dector.command[dector.nodeIndex]);
                dector.nodeIndex++;
                dector.readyToTurn = false;
                cout <<"the command of turn() has been carried out"<<endl;
            }

        }

//        if(dector.readyToTurn) {
  //          if(dector.centre_y > dector.imageRows*2/3){
    //            turn(620000, dector.command[dector.nodeIndex]);
      //          dector.nodeIndex++;
        //        dector.readyToTurn = false;
          //  } else if(dector.centre_y > dector.imageRows/2){
//                turn(640000, dector.command[dector.nodeIndex]);
  //              dector.nodeIndex++;
    //            dector.readyToTurn = false;
      //      } else if(dector.centre_y > dector.imageRows/3){
        //        turn(840000, dector.command[dector.nodeIndex]);
          //      dector.nodeIndex++;
//                dector.readyToTurn = false;
  //          }
    //    }
        if(dector.commandStop) {
        if(dector.centre_y > dector.imageRows*2/3){
        turn(640000, 5);
        dector.commandStop = false;
        } else if(dector.centre_y > dector.imageRows/2){
        turn(740000, 5);
        dector.commandStop = false;
        } else if(dector.centre_y > dector.imageRows/3){
        turn(880000, 5);
        dector.commandStop = false;
        }
    }
    }
}

void setTimer(){
    sleep(1);
    while(true){
        double cur_time = clock();
        if(cur_time - lastTime >= (2*CLOCKS_PER_SEC/100)) {
            lastTime = cur_time;
            if(turned) {
                adjustDirection();
            } else {
                PIDControl();
            }
        }
    }
}

void turn(int time, int type){//4:turnRight 3:turnLeft 1:farword 5:stop
//    if(type == 1) {
  //   cout<<"type ==1"<<endl;  
  //   return;//farword, noting to do
   // }

    usleep(time);

    sLock.lock();
    stopPid = true;
    char send_buf_stop[8]="z 0 0;\r";
    if(UART_Send(fd,send_buf_stop,7) <= 0){
        printf("send data failed!\n");
    }
    if(type == 5){
        sLock.unlock();
        char * stateMsg = "s2e";
        write(clnt_sock, stateMsg, strlen(stateMsg));
        cout << "arrive in destination!!!" << endl;
//        if(dector.nodeIndex > 0 && dector.decode_value != 132 && dector.decode_value != 512 && dector.decode_value == dector.stopNum){
//            cout << "motor car unload !!!" << endl;
//            char send_buf_unload[8]="z 0 0;\r";
//            if(UART_Send(fd,send_buf_unload,7) <= 0){
//                printf("send data failed!\n");
//            }
//        }
        return;
    }
    cout << "motro car stop to turn!" << endl;

    usleep(300000);

    char send_buf_turn[10] = {'z',' ','5','0',' ','-','5','0',';','\r'};
    if(type == 4) {

        cout << "car turning right" << endl;
    } else if (type == 3) {

        cout << "car turning left" << endl;
    }

    if(UART_Send(fd,send_buf_turn,10) <= 0){
        printf("send data failed!\n");
    }
    sLock.unlock();

    usleep(769000);

    sLock.lock();
    if(UART_Send(fd,send_buf_stop,10) <= 0){
        printf("send data failed!\n");
    }
    cout << "car finished turning, now going" << endl;
    sLock.unlock();

//    usleep(1000000);
    stopPid = false;
    finishedTurn = true;
    turned = true;
}

void adjustDirection(){
    if(dector.position_err > 10) {
        char send_buf_stop[11]="z 20 -20;\r";
        if(UART_Send(fd,send_buf_stop,10) <= 0){
            printf("send data failed!\n");
        }
    } else if(dector.position_err < -10) {
        char send_buf_stop[11]="z -20 20;\r";
        if(UART_Send(fd,send_buf_stop,10) <= 0){
            printf("send data failed!\n");
        }
    } else{
        turned = false;
        char send_buf_stop[8]="z 0 0;\r";
        if(UART_Send(fd,send_buf_stop,7) <= 0){
            printf("send data failed!\n");
        }
    }
}

void PIDControl(){
    sLock.lock();
    if(stopPid){
        sLock.unlock();
    return;
    }
    if(dector.position_err != last_position_err){
      //  cout << "err_gap:" <<  << endl;
    }
    double err_gap = last_position_err - dector.position_err;
    last_position_err = dector.position_err;
    if(finishedTurn){
        pid_count++;
        if(pid_count == 120){
            pid_count = 0;
            finishedTurn = false;
        }
        speed = 30;
        Kd = 0;
        Kp = 0.25;
    } else {
        speed = 50;
        Kd = 4;
        Kp = 0.12;
    }
    D_value = (int)(Kp * dector.position_err - Kd * err_gap);
    if(D_value > MAX_D_VALUE){
    D_value = MAX_D_VALUE;
    } else if(D_value < (0 - MAX_D_VALUE)){
    D_value = 0 - MAX_D_VALUE;
    }

    int left_value = speed - D_value/2;
    int right_value = speed + D_value/2;

    char send_buf[10]="z 10 10;\r";
    send_buf[2] = 48 + left_value/10;
    send_buf[3] = 48 + (left_value - (send_buf[2] - 48)*10);
    send_buf[5] = 48 + right_value/10;
    send_buf[6] = 48 + (right_value - (send_buf[5] - 48)*10);
    if(send_buf[2]<= 49 ||send_buf[5]<=49){
        cout<< "one motor dose not work"<<endl;
    }
    if(UART_Send(fd,send_buf,9) <= 0){
        printf("send data failed!\n");
    }
    sLock.unlock();
}

/*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART_Open(int fd,char* port)
{

    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }

    //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(FALSE);
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}
/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/

void UART0_Close(int fd)
{
    close(fd);
}

/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
              options.c_cflag &= ~CRTSCTS;
              break;

        case 1 ://使用硬件流控制
              options.c_cflag |= CRTSCTS;
              break;
        case 2 ://使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
                     options.c_cflag |= CS5;
                     break;
        case 6    :
                     options.c_cflag |= CS6;
                     break;
        case 7    :
                 options.c_cflag |= CS7;
                 break;
        case 8:
                 options.c_cflag |= CS8;
                 break;
        default:
                 fprintf(stderr,"Unsupported data size\n");
                 return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB;
                 options.c_iflag &= ~INPCK;
                 break;
        case 'o':
        case 'O'://设置为奇校验
                 options.c_cflag |= (PARODD | PARENB);
                 options.c_iflag |= INPCK;
                 break;
        case 'e':
        case 'E'://设置为偶校验
                 options.c_cflag |= PARENB;
                 options.c_cflag &= ~PARODD;
                 options.c_iflag |= INPCK;
                 break;
        case 's':
        case 'S': //设置为空格
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break;
        default:
                 fprintf(stderr,"Unsupported parity\n");
                 return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
                 options.c_cflag &= ~CSTOPB; break;
        case 2:
                 options.c_cflag |= CSTOPB; break;
        default:
                       fprintf(stderr,"Unsupported stop bits\n");
                       return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}
/*******************************************************************
* 名称：                UART0_Init()
* 功能：                串口初始化
* 入口参数：        fd       :  文件描述符
*               speed  :  串口速度
*                              flow_ctrl  数据流控制
*               databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART_Init(char * path, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err, fd;

    fd = UART_Open(fd, path); //打开串口，返回文件描述符
    do{
        err = UART_Set(fd, speed, flow_ctrl, databits, stopbits, parity);
        printf("Set Port Exactly!\n");
    }while(FALSE == err || FALSE == fd);
    return fd;
}

/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    printf("fs_sel = %d\n",fs_sel);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
        printf("Sorry,I am wrong!");
        return FALSE;
    }
}

/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART_Send(int fd, char *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
//        printf("send data is %s\n",send_buf);
        return len;
    }
    else
    {

        tcflush(fd,TCOFLUSH);
        return FALSE;
    }

}
