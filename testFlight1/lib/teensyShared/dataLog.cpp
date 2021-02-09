#include <SD.h>

#include "dataLog.h"

void init_dataLog(File* dataFile) {

    char fileName[12];

    strcpy(fileName,"data.csv");

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcpy(fileName, "data999.csv");
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "data(number).csv to fileNameTemp"
            char fileNameTemp[10+strlen(iStr)];
            strcpy(fileNameTemp,"data");
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp,".csv");

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    Serial.println(fileName);
    *dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_TRUNC);
    dataFile->println("ax, ay, az, gx, gy, gz, mx, my, mz, hg_ax, hg_ay, hg_az, latitude, longitude, altitude, rocketState, GPS Position Lock, timeStamp");

}


void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState) {

    //TODO: make this just use one print
    //TODO: log GPS timestamps
    dataFile->print(data->ax, 4);
    dataFile->print(",");
    dataFile->print(data->ay, 4);
    dataFile->print(",");
    dataFile->print(data->az, 4);
    dataFile->print(",");
    dataFile->print(data->gx, 4);
    dataFile->print(",");
    dataFile->print(data->gy, 4);
    dataFile->print(",");
    dataFile->print(data->gz, 4);
    dataFile->print(",");
    dataFile->print(data->mx, 4);
    dataFile->print(",");
    dataFile->print(data->my, 4);
    dataFile->print(",");
    dataFile->print(data->mz, 4);
    dataFile->print(",");
    //!highG imu data
    dataFile->print(data->hg_ax, 4);
    dataFile->print(",");
    dataFile->print(data->hg_ay, 4);
    dataFile->print(",");
    dataFile->print(data->hg_az, 4);
    dataFile->print(",");

    dataFile->print(data->latitude, 6);
    dataFile->print(",");
    dataFile->print(data->longitude, 6);
    dataFile->print(",");
    dataFile->print(data->altitude, 6);
    dataFile->print(",");
    dataFile->print(rocketState);
    dataFile->print(",");
    dataFile->print(data->posLock);
    dataFile->print(",");
    dataFile->print(data->timeStamp);
    dataFile->print("\n");

    //Writing line of data to SD card
    dataFile->flush();
}
