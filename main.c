#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct acce_data {
    int fl;
    double x, y, z;
} acce_q;
struct gyro_data {
    int fl;
    double x, y, z;
} gyro_q;
struct magn_data {
    int fl;
    double x, y, z;
} magn_q;
struct ahrs_data {
    int fl;
    double x, y, z;
} ahrs_q;
struct pres_data {
    int fl;
    double p;
} pres_q;

struct imu_data {//imu.txt文件需要输出的数据格式要求
    double ts;
    double accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, px, ry, yz, pr;
    int acfl, gyfl, mafl, ahfl, prfl;
} imu_data[200];
/////////////////////////////////////////////////////////////////////////////////////////
int imt = 0;
int imh = 0;

int flag = 0;
FILE *wf = NULL;
FILE *wifi = NULL;
FILE *gnss = NULL;

void balance(int type, int b, int t)//balance函数用作差值处理
{
    //由于数据采集频率不同，应做插值处理（采用线性插值），将所有数据频率均变为200HZ。
    int i, num;

    num = (t + 200 - b) % 200;
    for (i = b; i != t; i++, i %= 200) {
        if (i == b)
            continue;
        switch (type) {
        case 0:
            imu_data[i].accx = (imu_data[b].accx + imu_data[t].accx) * (((i + 200) - b) % 200) / num;
            imu_data[i].accy = (imu_data[b].accy + imu_data[t].accy) * (((i + 200) - b) % 200) / num;
            imu_data[i].accz = (imu_data[b].accz + imu_data[t].accz) * (((i + 200) - b) % 200) / num;
            break;
        case 1:
            imu_data[i].gyrx = (imu_data[b].gyrx + imu_data[t].gyrx) * (((i + 200) - b) % 200) / num;
            imu_data[i].gyry = (imu_data[b].gyry + imu_data[t].gyry) * (((i + 200) - b) % 200) / num;
            imu_data[i].gyrz = (imu_data[b].gyrz + imu_data[t].gyrz) * (((i + 200) - b) % 200) / num;
            break;
        case 2:
            imu_data[i].magx = (imu_data[b].magx + imu_data[t].magx) * (((i + 200) - b) % 200) / num;
            imu_data[i].magy = (imu_data[b].magy + imu_data[t].magy) * (((i + 200) - b) % 200) / num;
            imu_data[i].magz = (imu_data[b].magz + imu_data[t].magz) * (((i + 200) - b) % 200) / num;
            break;
        case 3:
            imu_data[i].px = (imu_data[b].px + imu_data[t].px) * (((i + 200) - b) % 200) / num;
            imu_data[i].ry = (imu_data[b].ry + imu_data[t].ry) * (((i + 200) - b) % 200) / num;
            imu_data[i].yz = (imu_data[b].yz + imu_data[t].yz) * (((i + 200) - b) % 200) / num;
            break;
        case 4:
            imu_data[i].pr = (imu_data[b].pr + imu_data[t].pr) * (((i + 200) - b) % 200) / num;
            break;
        }
    }
}


void write(int b, int t)//这里只是一个把数据写入文件的函数
{
    int i;
    for (i = b; i != t; i++, i %= 200) {
        fprintf(wf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
            imu_data[i].ts, imu_data[i].accx, imu_data[i].accy, imu_data[i].accz,
            imu_data[i].gyrx, imu_data[i].gyry, imu_data[i].gyrz, imu_data[i].magx,
            imu_data[i].magy, imu_data[i].magz, imu_data[i].px, imu_data[i].ry,
            imu_data[i].yz, imu_data[i].pr);
    }
}


void record(void)//实现100条写一次文件
{
    int i;
    int b, t;
    int bf = 200;

    b = 0, t = 0;
    for (i = imh; i != imt; i++, i %= 200) {
        if (imu_data[i].acfl != 0) {
            if (t != b) {
                balance(0, b, i);
            }
            b = i;
        }
        t = i;
    }
    bf = b;
    b = 0;
    t = 0;
    for (i = imh; i != imt; i++, i %= 200) {
        if (imu_data[i].gyfl != 0) {
            if (t != b) {
                balance(1, b, i);
            }
            b = i;
        }
        t = i;
    }
    if ((b - imh > 0 && bf - imh < 0)|| (b - imh < bf - imh)) {
        bf = b;
    }
    for (i = imh; i != imt; i++, i %= 200) {
        if (imu_data[i].mafl != 0) {
            if (t != b) {
                balance(2, b, i);
            }
            b = i;
        }
        t = i;
    }
    if ((b - imh > 0 && bf - imh < 0) || (b - imh < bf - imh)) {
        bf = b;
    }
    for (i = imh; i != imt; i++, i %= 200) {
        if (imu_data[i].ahfl != 0) {
            if (t != b) {
                balance(2, b, i);
            }
            b = i;
        }
        t = i;
    }
    if ((b - imh > 0 && bf - imh < 0)|| (b - imh < bf - imh)) {
        bf = b;
    }
    for (i = imh; i != imt; i++, i %= 200) {
        if (imu_data[i].prfl != 0) {
            if (t != b) {
                balance(4, b, i);
            }
            b = i;
        }
        t = i;
    }
    if ((b - imh > 0 && bf - imh < 0)|| (b - imh < bf - imh)) {
        bf = b;
    }
    write(imh, bf);
    imh = bf;
}


void imu_process(double ts)//imu函数
{
    imu_data[imt].ts = ts;
    imu_data[imt].acfl = acce_q.fl;
    imu_data[imt].accx = acce_q.x;
    imu_data[imt].accy = acce_q.y;
    imu_data[imt].accz = acce_q.z;
    imu_data[imt].gyfl = gyro_q.fl;
    imu_data[imt].gyrx = gyro_q.x;
    imu_data[imt].gyry = gyro_q.y;
    imu_data[imt].gyrz = gyro_q.z;
    imu_data[imt].mafl = magn_q.fl;
    imu_data[imt].magx = magn_q.x;
    imu_data[imt].magy = magn_q.y;
    imu_data[imt].magz = magn_q.z;
    imu_data[imt].ahfl = ahrs_q.fl;
    imu_data[imt].px   = ahrs_q.x;
    imu_data[imt].ry   = ahrs_q.y;
    imu_data[imt].yz   = ahrs_q.z;
    imu_data[imt].prfl = pres_q.fl;
    imu_data[imt].pr   = pres_q.p;
    imt++;
    imt %= 200;
    if (imt >= imh + 100 || ((imt < imh ) && ((200 - imh) + imt >= 100))) {
        record();
    }
    acce_q.fl = 0;
    gyro_q.fl = 0;
    magn_q.fl = 0;
    ahrs_q.fl = 0;
    pres_q.fl = 0;
}


void wifi_process(double ts, unsigned long long macd, char *rss)//wifi函数
{
    fprintf(wifi, "%lf,%lu,%s", ts, macd, rss);
}


void gnss_process(double ad, double ts, char *info)//gnss函数
{
    static double tsr = -1;
    static double adr = 0;

    if (tsr - ts < 0.00001 && tsr - ts > -0.00001) {
        return;
    }
    tsr = ts;
    fprintf(gnss, "%d,%s\n", (int)(ad - adr), info);
}

int main()//main函数
{
    FILE *fp = fopen("D:\\code_by_zwy\\final_test_zwy\\logfile.txt", "r");
    //打开第二个测试文件logfile.txt
    char str[512] = {0};
    char *split = NULL;
    char *name = NULL;
    double ts = 0;
    double tst;
    double at;

    wf = fopen("imu_logfile.txt", "w");
    wifi = fopen("wifi_logfile.txt", "w");
    gnss = fopen("gnss_logfile.txt", "w");

    //三个输出文件imu.txt,wifi.txt,gnss.txt
    if (fp == NULL || wf == NULL || wifi == NULL || gnss == NULL) {
        //打开失败
        return -1;
    }

    //接下来读数据，判断一下读入的数据类型
    //考虑使用函数strtok分解字符串为一组字符串
    while(fgets(str, 512, fp) != NULL) {
        //char *fgets(char *str, int n, FILE *stream);
        //一系列的if和elseif语句判断读入数据的类型
        if (str[0] == '%') {//%开头的是注释，continue掉，继续读下面数据
            continue;
        } else if(strlen(str) > 1){//开始处理有效数据
        //根据分号分割有效数据，name记录4个字母的数据类型
            name = strtok(str, ";");
            split = strtok(NULL, ";");
            sscanf(split, "%lf", &at);
            split = strtok(NULL, ";");
            sscanf(split, "%lf", &tst);
            if (ts > tst || ts < 0.00001) {
                ts = tst;
            } else if (ts < tst + 0.03) {
                imu_process(ts);
                ts = tst;
            }//end else if
            //接下来if语句对比存取的name和物种类型ACCE、GYRO、MAGN、PRES、AHRS
            //判断读入的数据类型
            if (strcmp(name, "ACCE") == 0) {
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &acce_q.x);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &acce_q.y);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &acce_q.z);
                flag |= 0x1;
                acce_q.fl = 1;
            } else if (strcmp(name, "GYRO") == 0) {
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &gyro_q.x);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &gyro_q.y);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &gyro_q.z);
                flag |= 0x2;
                gyro_q.fl = 1;
            } else if (strcmp(name, "MAGN") == 0) {
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &magn_q.x);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &magn_q.y);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &magn_q.z);
                flag |= 0x4;
                magn_q.fl = 1;
            } else if (strcmp(name, "AHRS") == 0) {
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &ahrs_q.x);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &ahrs_q.y);
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &ahrs_q.z);
                flag |= 0x8;
                ahrs_q.fl = 1;
            } else if (strcmp(name, "PRES") == 0) {
                split = strtok(NULL, ";");
                sscanf(split, "%lf", &pres_q.p);
                flag |= 0x10;
                pres_q.fl = 1;
            } else if (strcmp(name, "WIFI") == 0) {
                unsigned long long macd = 0;
                unsigned int temp[6] = {0};
                char *ntr = NULL;
                split = strtok(NULL, ";");
                split = strtok(NULL, ";");
                sscanf(split, "%x:%x:%x:%x:%x:%x",
                    &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
                macd = ((((temp[5] * 256 + temp[4]) * 256 + temp[3]) * 256 + temp[2]) * 256 + temp[1]) * 256 + temp[0];
                while (split != NULL) {
                    ntr = split;
                    split = strtok(NULL, ";");
                }
                wifi_process(tst, macd, ntr);
            } else if (strcmp (name, "GNSS") == 0) {
                int i;
                split = strtok(NULL, "\n");
                for (i = 0; split[i]; i++) {
                    if (split[i] == ';') {
                        split[i] = ',';
                    }
                }
                gnss_process(at, ts, split);
            }//end else if(strcmp (name, "GNSS") == 0)
        }//end else if(strlen(str) > 1)
    }//end while()  
    fclose(fp);
    fclose(wf);
    fclose(wifi);
    fclose(gnss);
    return 0;
}//end int main()